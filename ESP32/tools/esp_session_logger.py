#!/usr/bin/env python3
"""
ESP32 session logger
- Connects to the ESP32 Wi-Fi console server (same protocol as wifi_console_client.py)
- Watches serial log lines for balancer start/stop events
- When a BALANCER start is detected it begins recording; stops on BALANCER stop or auto-stop
- Parses common numeric log lines (DRIVE DBG, MOTOR DBG, SETDRIVE, fall-stop) into time series
- Saves raw log, CSVs and generates plots (requires matplotlib)

Usage: python esp_session_logger.py <host> [port] [--outdir DIR] [--no-plots]

Run, then place the robot vertical and start the balancer with your controller.
The script will automatically record the session between BALANCER start and stop.
"""

import socket
import sys
import threading
import time
import os
import re
import argparse
from collections import defaultdict
import bisect

try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False


def make_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except Exception:
        pass
    return s


class ESPLogger:
    def __init__(self, host, port, outdir, no_plots=False):
        self.host = host
        self.port = port
        self.sock = None
        self.reader_thr = None
        self.reader_stop = threading.Event()
        self.connected = threading.Event()
        self.outdir = outdir
        self.no_plots = no_plots
        os.makedirs(outdir, exist_ok=True)

        # state
        self.recording = False
        self.session_start_ts = None
        self.session_lines = []  # (t, line)
        # parsed series: name -> list of (t, value)
        self.series = defaultdict(list)

        # regex patterns (common ones from firmware logs)
        self.re_start = re.compile(r"BALANCER: started")
        self.re_stop = re.compile(r"BALANCER: stopped|BALANCER: auto-stopped")
        self.re_drive_dbg = re.compile(r"DRIVE DBG .*pitch_setpoint=([\-\d\.]+)deg .*pid_in=([\-\d\.]+)deg .*pid_out=([\-\d\.]+)")
        # Newer firmware adds BALANCER_DBG with pid/command info (rate-limited)
        self.re_balancer_dbg = re.compile(r"BALANCER_DBG pitch=([\-\d\.]+)deg pitch_rate=([\-\d\.]+)deg/s pid_in=([\-\d\.]+) pid_out=([\-\d\.]+) cmd_pre_slew=([\-\d\.]+) cmd_after_slew=([\-\d\.]+) last_cmd=([\-\d\.]+)")
        self.re_motor_left = re.compile(r"MOTOR DBG LEFT ([\-\d\.]+)")
        self.re_motor_right = re.compile(r"MOTOR DBG RIGHT ([\-\d\.]+)")
        self.re_setdrive = re.compile(r"SETDRIVE: v_req=([\-\d\.]+) w_req=([\-\d\.]+)")
        self.re_fall = re.compile(r"BALANCER: auto-stopped.*pitch=([\-\d\.]+)deg rate=([\-\d\.]+)deg/s")
        # optional pitch lines
        self.re_pitch = re.compile(r"pitch=([\-\d\.]+)deg")

    def connect(self, timeout=5):
        backoff = 1.0
        while True:
            try:
                s = make_socket()
                s.settimeout(timeout)
                s.connect((self.host, self.port))
                s.settimeout(None)
                self.sock = s
                self.connected.set()
                print(f"Connected to {self.host}:{self.port}")
                self.reader_stop.clear()
                self.reader_thr = threading.Thread(target=self._reader, daemon=True)
                self.reader_thr.start()
                return True
            except Exception as e:
                print(f"Connect failed: {e}; retrying in {backoff}s...")
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 16.0)

    def close(self):
        try:
            self.reader_stop.set()
            if self.sock:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                self.sock.close()
        finally:
            self.sock = None
            self.connected.clear()

    def send_cmd(self, s):
        if not s.endswith('\n'):
            s = s + '\n'
        b = s.encode('utf-8')
        try:
            if not self.sock:
                raise OSError('not connected')
            self.sock.sendall(b)
            return True
        except Exception as e:
            print('[Send failed]', e)
            return False

    def _record_line(self, line):
        t = time.time()
        self.session_lines.append((t, line))
        # parse patterns
        m = self.re_drive_dbg.search(line)
        if m:
            pitch_setpoint = float(m.group(1))
            pid_in = float(m.group(2))
            pid_out = float(m.group(3))
            self.series['pitch_setpoint'].append((t, pitch_setpoint))
            self.series['pid_in'].append((t, pid_in))
            self.series['pid_out'].append((t, pid_out))
        m = self.re_motor_left.search(line)
        if m:
            val = float(m.group(1))
            self.series['motor_left'].append((t, val))
        m = self.re_motor_right.search(line)
        if m:
            val = float(m.group(1))
            self.series['motor_right'].append((t, val))
        m = self.re_setdrive.search(line)
        if m:
            v = float(m.group(1)); w = float(m.group(2))
            self.series['setdrive_v'].append((t, v))
            self.series['setdrive_w'].append((t, w))
        m = self.re_fall.search(line)
        if m:
            pitch = float(m.group(1)); rate = float(m.group(2))
            self.series['fall_pitch'].append((t, pitch))
            self.series['fall_rate'].append((t, rate))
        # Parse BALANCER_DBG if present (contains pid and command pre/post-slew)
        m = self.re_balancer_dbg.search(line)
        if m:
            b_pitch = float(m.group(1))
            b_pitch_rate = float(m.group(2))
            b_pid_in = float(m.group(3))
            b_pid_out = float(m.group(4))
            b_cmd_pre = float(m.group(5))
            b_cmd_after = float(m.group(6))
            b_last_cmd = float(m.group(7))
            self.series['bal_pitch'].append((t, b_pitch))
            self.series['bal_pitch_rate'].append((t, b_pitch_rate))
            self.series['bal_pid_in'].append((t, b_pid_in))
            self.series['bal_pid_out'].append((t, b_pid_out))
            self.series['bal_cmd_pre'].append((t, b_cmd_pre))
            self.series['bal_cmd_after'].append((t, b_cmd_after))
            self.series['bal_last_cmd'].append((t, b_last_cmd))
        # try generic pitch
        if 'pitch' in line and 'deg' in line:
            m = self.re_pitch.search(line)
            if m:
                p = float(m.group(1))
                self.series['pitch_any'].append((t, p))

    def _reader(self):
        buf = b''
        try:
            while not self.reader_stop.is_set():
                try:
                    data = self.sock.recv(1024)
                except OSError as e:
                    print('[Reader error]', e)
                    break
                if not data:
                    print('[Disconnected]')
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    try:
                        s = line.decode('utf-8', errors='ignore')
                    except Exception:
                        s = repr(line)
                    s = s.strip()
                    # print live to terminal
                    print(s)
                    # session management
                    if not self.recording and self.re_start.search(s):
                        # start recording
                        self.recording = True
                        self.session_start_ts = time.time()
                        self.session_lines = []
                        self.series = defaultdict(list)
                        print('[Session] BALANCER start detected, recording ON')
                    if self.recording:
                        self._record_line(s)
                    if self.recording and self.re_stop.search(s):
                        print('[Session] BALANCER stop/auto-stop detected, recording OFF')
                        self._finish_session()
        finally:
            self.close()

    def _finish_session(self):
        # write raw log
        t0 = int(self.session_start_ts or time.time())
        basename = f"session_{time.strftime('%Y%m%d_%H%M%S', time.localtime(t0))}"
        raw_path = os.path.join(self.outdir, basename + '.log')
        with open(raw_path, 'w', encoding='utf-8') as f:
            for t, line in self.session_lines:
                f.write(f"{t:.6f} {line}\n")
        print(f'[Session] Raw log saved: {raw_path}')
        # write CSV for each series
        for name, samples in self.series.items():
            if not samples:
                continue
            csv_path = os.path.join(self.outdir, f"{basename}_{name}.csv")
            with open(csv_path, 'w', encoding='utf-8') as f:
                f.write('t,value\n')
                for t, v in samples:
                    f.write(f"{t:.6f},{v}\n")
            print(f'[Session] CSV saved: {csv_path}')
        # write a combined, time-aligned CSV for all series
        try:
            combined_path = os.path.join(self.outdir, f"{basename}_combined.csv")
            self._write_combined_csv(combined_path)
            print(f'[Session] Combined CSV saved: {combined_path}')
        except Exception as e:
            print('[Session] error writing combined CSV:', e)
        # produce plots unless disabled
        if not self.no_plots:
            if not HAS_MPL:
                print('[Plot] matplotlib not available; skipping plots. Install via: pip install matplotlib')
            else:
                try:
                    self._make_plots(basename)
                except Exception as e:
                    print('[Plot] error generating plots:', e)
        # reset recording flag
        self.recording = False
        self.session_start_ts = None
        self.session_lines = []
        self.series = defaultdict(list)

    def _make_plots(self, basename):
        # improved plotting: time-aligned, shared x-axis, interpolation
        outpng = os.path.join(self.outdir, basename + '.png')
        # collect available series keys
        keys = [k for k, v in self.series.items() if v]
        if not keys:
            print('[Plot] no series available for plotting')
            return
        n = len(keys)
        fig, axs = plt.subplots(n, 1, figsize=(10, 2.5 * n), squeeze=False, sharex=True)
        axs = axs.flatten()
        # build unioned timebase
        union_ts = sorted({t for samples in self.series.values() for (t, _) in samples})
        if not union_ts:
            print('[Plot] no timestamps found')
            return
        start_t = union_ts[0]
        rel_ts = [t - start_t for t in union_ts]
        for i, key in enumerate(keys):
            samples = self.series.get(key, [])
            vals = [self._interp_value(samples, t) for t in union_ts]
            axs[i].plot(rel_ts, vals, label=key)
            axs[i].set_title(key)
            axs[i].legend(loc='upper right')
            axs[i].grid(True)
        axs[-1].set_xlabel('time (s)')
        plt.tight_layout()
        fig.savefig(outpng)
        plt.close(fig)
        print(f'[Plot] Saved combined plot: {outpng}')

    def _interp_value(self, samples, t):
        """Interpolate value for time t given samples [(ti, vi), ...].
        If outside range, return nearest endpoint value."""
        if not samples:
            return float('nan')
        ts = [s[0] for s in samples]
        vs = [s[1] for s in samples]
        if t <= ts[0]:
            return vs[0]
        if t >= ts[-1]:
            return vs[-1]
        i = bisect.bisect_right(ts, t)
        # interpolate between i-1 and i
        t0, v0 = ts[i-1], vs[i-1]
        t1, v1 = ts[i], vs[i]
        if t1 == t0:
            return v0
        ratio = (t - t0) / (t1 - t0)
        return v0 + ratio * (v1 - v0)

    def _write_combined_csv(self, path):
        # create union of timestamps and interpolate each series
        keys = [k for k, v in self.series.items() if v]
        if not keys:
            raise RuntimeError('no series to combine')
        union_ts = sorted({t for samples in self.series.values() for (t, _) in samples})
        start_t = union_ts[0]
        with open(path, 'w', encoding='utf-8') as f:
            header = ['t_rel'] + keys
            f.write(','.join(header) + '\n')
            for t in union_ts:
                row = [f"{t - start_t:.6f}"]
                for k in keys:
                    val = self._interp_value(self.series.get(k, []), t)
                    if isinstance(val, float) and (val != val):
                        row.append('')
                    else:
                        row.append(f"{val}")
                f.write(','.join(row) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('host')
    parser.add_argument('port', nargs='?', type=int, default=2333)
    parser.add_argument('--outdir', default='esp_sessions')
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()

    logger = ESPLogger(args.host, args.port, args.outdir, args.no_plots)
    try:
        logger.connect()
    except KeyboardInterrupt:
        print('Interrupted before connect')
        return

    print('Logger running. Place robot vertical, start balancer with controller to record session.')
    print('Type commands to send to ESP (will be forwarded). Ctrl-C to quit.')
    # simple stdin loop to forward user commands while connected
    try:
        while True:
            line = sys.stdin.readline()
            if not line:
                break
            line = line.rstrip('\n')
            if line == 'quit' or line == 'exit':
                break
            if line:
                ok = logger.send_cmd(line)
                if not ok:
                    print('[Send] failed')
    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        print('Closed')


if __name__ == '__main__':
    main()
