#!/usr/bin/env python3
"""
tools/capture_balancer_dbg.py

Connect to the Wi‑Fi console, wait for the balancer to start, capture DRIVE DBG
and BALANCER_DBG lines until the balancer stops, then save a CSV and a plot
showing pitch, accel/gyro (when available) and motor command traces.

Usage:
  python tools/capture_balancer_dbg.py --host 192.168.1.29 --port 2333 -o balancer_log.txt

"""
import argparse
import re
import socket
import sys
import time
from collections import defaultdict

import numpy as np
import pandas as pd


DRIVE_RE = re.compile(r"DRIVE DBG t=(?P<t>\d+)ms .*?tgtV=(?P<tgtV>[-0-9.eE]+) .*?filtV=(?P<filtV>[-0-9.eE]+) .*?pitch_sp=(?P<pitch_sp>[-0-9.eE]+)deg .*?pitch_sp_rate=(?P<pitch_sp_rate>[-0-9.eE]+)deg/s .*?pid_in=(?P<pid_in>[-0-9.eE]+)deg .*?pid_rate=(?P<pid_rate>[-0-9.eE]+)deg/s .*?pid_out=(?P<pid_out>[-0-9.eE]+)")
BAL_RE = re.compile(
    r"BALANCER_DBG t=(?P<t>\d+)ms .*?pitch=(?P<pitch>[-0-9.eE]+)deg "
    r"pitch_rate=(?P<pitch_rate>[-0-9.eE]+)deg/s .*?pid_in=(?P<pid_in>[-0-9.eE]+)deg "
    r"pid_out=(?P<pid_out>[-0-9.eE]+) .*?cmd_pre_slew=(?P<cmd_pre_slew>[-0-9.eE]+) "
    r"cmd_after_slew=(?P<cmd_after_slew>[-0-9.eE]+) .*?last_cmd=(?P<last_cmd>[-0-9.eE]+) "
    r"ax=(?P<ax>[-0-9.eE]+) ay=(?P<ay>[-0-9.eE]+) az=(?P<az>[-0-9.eE]+) "
    r"gx=(?P<gx>[-0-9.eE]+)deg/s gy=(?P<gy>[-0-9.eE]+)deg/s gz=(?P<gz>[-0-9.eE]+)deg/s"
)


def parse_line(line, rows):
    m = DRIVE_RE.search(line)
    if m:
        d = {k: float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['drive'][t].update(d)
        return
    m = BAL_RE.search(line)
    if m:
        d = {k: float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['bal'][t].update(d)
        return
    # try simple CSV-style imu line (legacy TUNING), detect by commas and numeric start
    if ',' in line:
        parts = [p.strip() for p in line.split(',')]
        try:
            t0 = float(parts[0])
            # guess layout similar to TUNING CSV: timestamp_ms, pitch_deg, pitch_rad, pitch_rate_deg, pitch_rate_rad, ax,ay,az,gx,gy,gz,...
            if len(parts) >= 11:
                t = int(t0)
                ax = float(parts[5])
                ay = float(parts[6])
                az = float(parts[7])
                gx = float(parts[8])
                gy = float(parts[9])
                gz = float(parts[10])
                rows['imu'][t] = {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host')
    p.add_argument('--port', type=int, default=2333)
    p.add_argument('-o', '--out', default='balancer_capture.txt')
    p.add_argument('--csv', default='balancer_capture.csv')
    p.add_argument('--png', default='balancer_capture.png')
    p.add_argument('--timeout', type=int, default=10, help='seconds to wait for start')
    p.add_argument('--read-csv', help='If provided, skip network capture and read an existing CSV')
    p.add_argument('--from-log', help='Parse a balancer_capture.txt log (BALANCER/DRIVE DBG lines) to CSV/plot')
    p.add_argument('--ffill', dest='ffill', action='store_true', default=True, help='Forward-fill sparse columns before plotting (default on)')
    p.add_argument('--no-ffill', dest='ffill', action='store_false', help='Disable forward-fill for plotting')
    p.add_argument('--ffill-limit', type=int, default=5000, help='Maximum ms to forward-fill')
    args = p.parse_args()

    # If user supplied --read-csv, skip live capture and only plot/analyze existing CSV.
    rows = {'drive': defaultdict(dict), 'bal': defaultdict(dict), 'imu': dict()}
    if args.read_csv:
        print('reading CSV for plotting:', args.read_csv)
        df = pd.read_csv(args.read_csv)
    elif args.from_log:
        print('parsing log for plotting:', args.from_log)
        with open(args.from_log, 'r', encoding='utf8', errors='ignore') as fh:
            for line in fh:
                parse_line(line.rstrip('\n'), rows)
        # build dataframe
        keys = set(rows['drive'].keys()) | set(rows['bal'].keys()) | set(rows['imu'].keys())
        if not keys:
            print('no parsed records found in log')
            return
        times = sorted(keys)
        recs = []
        for t in times:
            r = {'time_ms': t}
            r.update(rows['imu'].get(t, {}))
            r.update(rows['drive'].get(t, {}))
            r.update(rows['bal'].get(t, {}))
            recs.append(r)
        df = pd.DataFrame(recs).sort_values('time_ms')
        df.to_csv(args.csv, index=False)
        print('wrote CSV', args.csv)
    else:
        if not args.host:
            print('error: --host required for live capture (or use --read-csv)')
            return
        sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
        f = sock.makefile('r', encoding='utf8', errors='ignore')
        print('connected to', args.host, args.port)

        # wait for balancer started
        start_re = re.compile(r'BALANCER: started.*motors ENABLED', re.IGNORECASE)
        stop_re = re.compile(r'BALANCER: stopped.*motors DISABLED', re.IGNORECASE)
        started = False
        start_time = time.time()
        with open(args.out, 'w', encoding='utf8') as outfh:
            try:
                while True:
                    line = f.readline()
                    if not line:
                        time.sleep(0.01)
                        continue
                    line = line.rstrip('\n')
                    outfh.write(line + '\n')
                    outfh.flush()
                    if not started:
                        if start_re.search(line):
                            started = True
                            print('detected balancer start — begin capture')
                    else:
                        parse_line(line, rows)
                        if stop_re.search(line):
                            print('detected balancer stop — ending capture')
                            break
            except KeyboardInterrupt:
                print('capture interrupted by user')

        # build combined dataframe keyed by timestamp (ms)
        keys = set()
        for k in rows['drive'].keys():
            keys.add(k)
        for k in rows['bal'].keys():
            keys.add(k)
        for k in rows['imu'].keys():
            keys.add(k)
        if not keys:
            print('no parsed records found')
            return
        times = sorted(keys)
        recs = []
        for t in times:
            r = {'time_ms': t}
            r.update(rows['imu'].get(t, {}))
            r.update(rows['drive'].get(t, {}))
            r.update(rows['bal'].get(t, {}))
            recs.append(r)
        df = pd.DataFrame(recs)
        df = df.sort_values('time_ms')
        df.to_csv(args.csv, index=False)
        print('wrote CSV', args.csv)

    # If we performed live capture, the dataframe was already built above.
    if not args.read_csv:
        # build combined dataframe keyed by timestamp (ms)
        keys = set()
        for k in rows['drive'].keys():
            keys.add(k)
        for k in rows['bal'].keys():
            keys.add(k)
        for k in rows['imu'].keys():
            keys.add(k)
        if not keys:
            print('no parsed records found')
            return
        times = sorted(keys)
        recs = []
        for t in times:
            r = {'time_ms': t}
            r.update(rows['imu'].get(t, {}))
            r.update(rows['drive'].get(t, {}))
            r.update(rows['bal'].get(t, {}))
            recs.append(r)
        df = pd.DataFrame(recs)
        df = df.sort_values('time_ms')
        df.to_csv(args.csv, index=False)
        print('wrote CSV', args.csv)

    # plotting / cleanup
    try:
        import matplotlib.pyplot as plt

        # ensure expected columns exist
        expected = ['pitch', 'pitch_sp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'pid_out', 'cmd_pre_slew', 'cmd_after_slew', 'last_cmd', 'tgtV']
        for c in expected:
            if c not in df.columns:
                df[c] = np.nan

        # numeric conversion
        for c in df.columns:
            if c != 'time_ms':
                df[c] = pd.to_numeric(df[c], errors='coerce')

        # optionally forward-fill sparse values (limit in ms -> convert to row count approx)
        if args.ffill:
            # conservative: forward-fill without exceeding a duration. We can't easily map ms->rows reliably,
            # so use a simple forward-fill and then set large gaps back to NaN using time diffs.
            df = df.sort_values('time_ms')
            df = df.reset_index(drop=True)
            df_ff = df.fillna(method='ffill')
            # compute time gaps and invalidate fills that span bigger than ffill-limit
            dt = df['time_ms'].diff().fillna(0).astype(float)
            biggaps = dt > args.ffill_limit
            if biggaps.any():
                # for rows after a big gap, revert any forward-filled values that originated before the gap
                grp = biggaps.cumsum()
                df_out = df.copy()
                for name, g in df.groupby(grp):
                    idx = g.index
                    df_out.loc[idx] = df_ff.loc[idx]
                df = df_out
            else:
                df = df_ff

        # report sparsity
        nonnull = df.count()
        print('non-null counts:')
        print(nonnull[nonnull.index != 'time_ms'].sort_values(ascending=False))

        t = (df['time_ms'].to_numpy().astype(float) - df['time_ms'].iloc[0]) / 1000.0
        plt.figure(figsize=(10, 8))
        ax1 = plt.subplot(3, 1, 1)
        ax1.plot(t, df['pitch_sp'], label='pitch_sp', alpha=0.6)
        ax1.plot(t, df['pitch'], label='pitch (filtered)', alpha=0.9)
        ax1.set_ylabel('pitch (deg)')
        ax1.legend()

        ax2 = plt.subplot(3, 1, 2, sharex=ax1)
        ax2.plot(t, df['ax'], label='ax')
        ax2.plot(t, df['ay'], label='ay')
        ax2.plot(t, df['az'], label='az')
        # gyro might already be deg/s in logs; if values look small (<10) assume rad/s and convert
        gx = df['gx'].copy()
        gy = df['gy'].copy()
        gz = df['gz'].copy()
        # Heuristic: if median absolute value < 1, treat as rad/s and convert
        for arr in (gx, gy, gz):
            if np.nanmedian(np.abs(arr)) < 1.0:
                arr[:] = np.degrees(arr)
        ax2.plot(t, gx, label='gx (deg/s)')
        ax2.plot(t, gy, label='gy (deg/s)')
        ax2.plot(t, gz, label='gz (deg/s)')
        ax2.set_ylabel('accel / gyro')
        ax2.legend()

        ax3 = plt.subplot(3, 1, 3, sharex=ax1)
        ax3.plot(t, df['pid_out'], label='pid_out')
        # plot commands — show sparse points with markers for readability
        ax3.plot(t, df['tgtV'], label='tgtV')
        ax3.plot(t, df['cmd_after_slew'], marker='o', linestyle='None', label='cmd_after_slew')
        ax3.plot(t, df['last_cmd'], marker='x', linestyle='None', label='last_cmd')
        ax3.set_ylabel('commands')
        ax3.legend()

        plt.xlabel('time (s)')
        plt.tight_layout()
        plt.savefig(args.png, dpi=150)
        print('saved plot', args.png)
    except Exception as e:
        print('plot failed:', e)


if __name__ == '__main__':
    main()
