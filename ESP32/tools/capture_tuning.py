#!/usr/bin/env python3
# capture_tuning.py
# Simple Python serial capture utility (uses pyserial).

import argparse
import sys
import time
import socket
try:
    import serial
except Exception:
    serial = None

# plotting libs are optional; we'll import them when needed

parser = argparse.ArgumentParser(description='Capture serial output to file')
parser.add_argument('-p','--port',default='COM10')
parser.add_argument('-b','--baud',type=int,default=921600)
parser.add_argument('-o','--outfile',default=None)
parser.add_argument('-c','--cmd',default=None,help='Optional command to send once after opening the connection')
parser.add_argument('-s','--stop-string',default='TUNING: capture stopped (auto)',help='If seen in serial/tcp output, stop capture and exit')
parser.add_argument('--host',default=None,help='If provided, connect to WiFi console TCP host (use IP address)')
parser.add_argument('--tcp-port',type=int,default=2333,help='WiFi console TCP port (default: 2333)')
parser.add_argument('--duration',type=float,default=None,help='Capture duration in seconds; will send TUNING STOP after this interval')
args = parser.parse_args()

port = args.port
baud = args.baud
outfile = args.outfile
if outfile is None:
    outfile = f'tuning_capture_{time.strftime("%Y%m%d_%H%M%S")}.txt'

if args.host:
    print(f'Opening TCP {args.host}:{args.tcp_port}, writing to {outfile}')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((args.host, args.tcp_port))
    except Exception as e:
        print('Failed to open TCP socket:', e)
        sys.exit(2)
    # make non-blocking for recv
    sock.setblocking(False)
    ser = None
else:
    if serial is None:
        print('pyserial not available; install pyserial or use --host to connect via WiFi console')
        sys.exit(2)
    print(f'Opening {port} @ {baud}, writing to {outfile}')
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print('Failed to open serial port:', e)
        sys.exit(2)

# After opening the port/socket, wait a short moment for the device to settle
try:
    if ser:
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass
        time.sleep(0.6)
        try:
            ser.reset_input_buffer()
        except Exception:
            end = time.time() + 0.1
            while time.time() < end:
                ser.read(1024)
    else:
        # for TCP, give some time and drain initial data
        time.sleep(0.2)
        try:
            sock.recv(4096)
        except Exception:
            pass
except Exception:
    pass

# If an initial command was provided, defer sending it until we see a likely
# interactive prompt / main menu from the device. This avoids missing the
# command when the ESP32 is still booting and the serial task isn't ready.
pending_cmd = None
if args.cmd:
    pending_cmd = args.cmd
    if not pending_cmd.endswith('\n'):
        pending_cmd = pending_cmd + '\n'
    # the actual send will occur once we observe a menu-like line in the serial
    cmd_sent = False

# capture duration handling
capture_duration = args.duration
start_time = None
stop_sent = False

def _normalize_cmd(cmd: str) -> str:
    # Normalize legacy/alternate tuning commands to the current CLI
    c = cmd.strip()
    # Accept variants like 'TUNING STREAM START' -> 'TUNING START'
    if c.upper().startswith('TUNING STREAM START'):
        return 'TUNING START\n'
    if c.upper().startswith('TUNING STREAM STOP'):
        return 'TUNING STOP\n'
    return cmd

# normalize pending command so firmware receives expected form
if pending_cmd:
    pending_cmd = _normalize_cmd(pending_cmd)

def _derive_stop_cmd(start_cmd: str) -> str:
    s = start_cmd.strip().upper()
    if s.startswith('TUNING START'):
        return 'TUNING STOP\n'
    if s.startswith('TUNING STREAM START'):
        return 'TUNING STREAM STOP\n'
    # fallback: try generic STOP
    return None

def send_cmd_socket(s, cmd):
    try:
        s.sendall(cmd.encode('utf8'))
        return True
    except Exception:
        return False

# If we're connected via TCP and have a pending command, send it immediately
# (WiFi console does not reset the ESP on connect so immediate send is safe)
if pending_cmd and args.host:
    try:
        ok = send_cmd_socket(sock, pending_cmd)
        if ok:
            print('Sent initial command (immediate via TCP):', pending_cmd.strip())
            start_time = time.time()
        else:
            print('Failed to send initial command immediately via TCP')
    except Exception:
        print('Error sending immediate command via TCP')
    cmd_sent = True

with open(outfile, 'w', encoding='utf8') as f:
    try:
        if ser:
            # serial mode: behave as before but also wait for warmup/menu triggers
            while True:
                line = ser.readline()
                if not line:
                    continue
                try:
                    s = line.decode('utf8', errors='replace').rstrip('\r\n')
                except Exception:
                    s = str(line)
                print(s)
                f.write(s + '\n')
                f.flush()

                # auto-stop: if we've started capture and duration elapsed, send stop
                if start_time and capture_duration and not stop_sent and (time.time() - start_time) >= capture_duration:
                    stop_cmd = _derive_stop_cmd(pending_cmd) if pending_cmd else None
                    if stop_cmd:
                        try:
                            ser.write(stop_cmd.encode('utf8'))
                            ser.flush()
                            print('Sent auto-stop command (serial):', stop_cmd.strip())
                        except Exception as e:
                            print('Failed to send auto-stop (serial):', e)
                    stop_sent = True
                    stop_sent_time = time.time()

                if pending_cmd and not cmd_sent:
                    triggers = ['TUNING: warmup complete', '== Main Menu ==', 'IMU tasks started']
                    for trig in triggers:
                        if trig in s:
                            try:
                                ser.write(pending_cmd.encode('utf8'))
                                ser.flush()
                                print('Sent initial command (deferred):', args.cmd)
                                start_time = time.time()
                            except Exception as e:
                                print('Failed to send initial command:', e)
                            cmd_sent = True
                            break

                if args.stop_string and args.stop_string in s:
                    print('End marker seen, stopping capture')
                    break
                # if we already sent stop and saw nothing for a short grace period, exit
                if stop_sent and ('stop_sent_time' in locals()) and (time.time() - stop_sent_time) > 2.0:
                    break
        else:
            # TCP socket mode: non-blocking recv loop
            buf = b''
            while True:
                lines = []
                try:
                    data = sock.recv(4096)
                    if data:
                        buf += data
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            try:
                                lines.append(line.decode('utf8', errors='replace').rstrip('\r'))
                            except Exception:
                                lines.append(str(line))
                    else:
                        # no data, small sleep
                        time.sleep(0.01)
                except BlockingIOError:
                    time.sleep(0.01)
                except Exception:
                    time.sleep(0.01)

                if not lines:
                    continue

                for s in lines:
                    print(s)
                    f.write(s + '\n')
                    f.flush()

                    # auto-stop: if we've started capture and duration elapsed, send stop
                    if start_time and capture_duration and not stop_sent and (time.time() - start_time) >= capture_duration:
                        stop_cmd = _derive_stop_cmd(pending_cmd) if pending_cmd else None
                        if stop_cmd:
                            ok2 = send_cmd_socket(sock, stop_cmd)
                            if ok2:
                                print('Sent auto-stop command (TCP):', stop_cmd.strip())
                            else:
                                print('Failed to send auto-stop (TCP)')
                        stop_sent = True
                        stop_sent_time = time.time()

                    if pending_cmd and not cmd_sent:
                        # For serial we wait for warmup; for TCP (WiFi console) the ESP isn't reset
                        # when connecting, so don't wait for warmup — send once we see menu/IMU ready.
                        triggers = ['== Main Menu ==', 'IMU tasks started']
                        for trig in triggers:
                            if trig in s:
                                ok = send_cmd_socket(sock, pending_cmd)
                                if ok:
                                    print('Sent initial command (deferred):', args.cmd)
                                    start_time = time.time()
                                else:
                                    print('Failed to send initial command via TCP')
                                cmd_sent = True
                                break

                    if args.stop_string and args.stop_string in s:
                        print('End marker seen, stopping capture')
                        raise KeyboardInterrupt
                    # if we already sent stop and saw nothing for a short grace period, exit
                    if stop_sent and ('stop_sent_time' in locals()) and (time.time() - stop_sent_time) > 2.0:
                        raise KeyboardInterrupt
    except KeyboardInterrupt:
        print('\nCapture stopped by user')
    finally:
        try:
            if ser:
                ser.close()
            else:
                sock.close()
        except Exception:
            pass

# After closing the connection, attempt to parse the captured file and plot
def _try_plot_capture(csv_path: str):
    try:
        import pandas as pd
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception:
        print('Plotting skipped: install pandas and matplotlib (pip install pandas matplotlib)')
        return

    print('Parsing capture for plotting:', csv_path)
    rows = []
    with open(csv_path, 'r', encoding='utf8', errors='ignore') as fh:
        for line in fh:
            s = line.strip()
            if not s:
                continue
            if ',' not in s:
                continue
            for p in ('TUNING STREAM:', 'TUNING:', 'TUNING STREAM', 'TUNING'):
                if s.startswith(p):
                    s = s[len(p):].strip()
            parts = [t.strip() for t in s.split(',')]
            # require first token to be numeric (timestamp)
            try:
                _ = float(parts[0])
            except Exception:
                continue
            nums = []
            for t in parts:
                try:
                    nums.append(float(t))
                except Exception:
                    nums.append(np.nan)
            if len(nums) < 6:
                continue
            rows.append(nums)

    if not rows:
        print('No CSV-like rows found in', csv_path)
        return

    maxcols = max(len(r) for r in rows)
    cols = [f'f{i}' for i in range(maxcols)]
    padded = [r + [np.nan] * (maxcols - len(r)) for r in rows]
    df = pd.DataFrame(padded, columns=cols)

    ts = df['f0'].astype(float)
    t0 = ts.iloc[0]
    time_s = (ts - t0) / 1000.0

    pitch = df['f1'] if 'f1' in df else df['f0']

    ax = df['f5'] if 'f5' in df else None
    ay = df['f6'] if 'f6' in df else None
    az = df['f7'] if 'f7' in df else None

    accel_pitch_deg = None
    if ax is not None and ay is not None and az is not None:
        axv = ax.astype(float).to_numpy()
        ayv = ay.astype(float).to_numpy()
        azv = az.astype(float).to_numpy()
        accel_pitch_rad = np.arctan2(axv, np.sqrt(ayv * ayv + azv * azv))
        accel_pitch_deg = np.degrees(accel_pitch_rad)

    gyro_integ_deg = None
    if 'f4' in df:
        rate_rad = df['f4'].astype(float).to_numpy()
        ts_arr = time_s.to_numpy()
        dt = np.diff(ts_arr, prepend=ts_arr[0])
        integ_rad = np.cumsum(rate_rad * dt)
        gyro_integ_deg = np.degrees(integ_rad)

    import matplotlib.pyplot as plt
    import numpy as np

    # Prepare three stacked panels for clearer comparison
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    # Panel 1: filter pitch vs accel-smoothed, plot filter last so it's on top
    if accel_pitch_deg is not None:
        # smooth accel for visual clarity
        try:
            accel_sm = pd.Series(accel_pitch_deg).rolling(window=9, min_periods=1, center=True).mean().to_numpy()
        except Exception:
            accel_sm = accel_pitch_deg
        ax1.plot(time_s, accel_sm, color='orange', alpha=0.9, linewidth=1.2, label='accel-smoothed')
    else:
        accel_sm = None

    if pitch is not None:
        ax1.plot(time_s, pitch.astype(float), color='blue', linewidth=2.2, zorder=5, label='filter-pitch')

    if accel_sm is not None and pitch is not None:
        diff = pitch.astype(float).to_numpy() - accel_sm
        ax1.plot(time_s, diff, color='gray', alpha=0.6, linewidth=0.8, label='filter - accel')

    ax1.set_ylabel('pitch (deg)')
    ax1.set_title('Filter vs Accel (filter plotted on top)')
    ax1.legend(loc='upper left')

    # Panel 2: raw accel-derived (transparent) and smoothed emphasized
    if accel_pitch_deg is not None:
        ax2.plot(time_s, accel_pitch_deg, color='orange', alpha=0.12, linewidth=0.6, label='accel-raw')
        if accel_sm is not None:
            ax2.plot(time_s, accel_sm, color='orange', alpha=0.95, linewidth=1.4, label='accel-smoothed')
    ax2.set_ylabel('accel pitch (deg)')
    ax2.set_title('Accel-derived pitch (raw + smoothed)')
    ax2.legend(loc='upper left')

    # Panel 3: gyro-integrated with linear drift estimate
    if gyro_integ_deg is not None:
        ax3.plot(time_s, gyro_integ_deg, color='green', linewidth=1.4, label='gyro-integrated')
        try:
            tarr = time_s.to_numpy()
            p = np.polyfit(tarr, gyro_integ_deg, 1)
            slope = p[0]
            fit = np.polyval(p, tarr)
            ax3.plot(time_s, fit, color='green', linestyle='--', alpha=0.7, label=f'linear fit ({slope:.4f} deg/s)')
        except Exception:
            pass
    ax3.set_ylabel('gyro integ (deg)')
    ax3.set_xlabel('time (s)')
    ax3.set_title('Gyro-integrated pitch and drift')
    ax3.legend(loc='upper left')

    plt.tight_layout()
    outpng = csv_path.rsplit('.', 1)[0] + '.png'
    try:
        plt.savefig(outpng, dpi=150)
        plt.close()
        print('Plot saved to', outpng)
    except Exception as e:
        print('Failed to save plot:', e)

try:
    _try_plot_capture(outfile)
except Exception:
    pass
