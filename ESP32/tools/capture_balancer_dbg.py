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


DRIVE_RE = re.compile(r"DRIVE DBG t=(?P<t>\d+)ms.*?tgtV=(?P<tgtV>[-0-9.eE]+).*?filtV=(?P<filtV>[-0-9.eE]+).*?pitch_setpoint=(?P<pitch_setpoint>[-0-9.eE]+)deg.*?pitch_setpoint_rate=(?P<pitch_setpoint_rate>[-0-9.eE]+)deg/s.*?pid_in=(?P<pid_in_drive>[-0-9.eE]+)deg.*?pid_rate=(?P<pid_rate_drive>[-0-9.eE]+)deg/s.*?pid_out=(?P<pid_out_drive>[-0-9.eE]+)")
SETDRIVE_RE = re.compile(r"SETDRIVE: t=(?P<t>\d+)ms v_req=(?P<v_req>[-0-9.eE]+) w_req=(?P<w_req>[-0-9.eE]+)")
BAL_RE = re.compile(
    r"BALANCER_DBG t=(?P<t>\d+)ms.*?pitch=(?P<pitch>[-0-9.eE]+)deg.*?pid_in=(?P<pid_in>[-0-9.eE]+)deg.*?pid_out=(?P<pid_out>[-0-9.eE]+).*?iterm=(?P<iterm>[-0-9.eE]+).*?cmd=(?P<cmd>[-0-9.eE]+).*?lat=(?P<lat>\d+)us.*?ax=(?P<ax>[-0-9.eE]+).*?ay=(?P<ay>[-0-9.eE]+).*?az=(?P<az>[-0-9.eE]+).*?gx=(?P<gx>[-0-9.eE]+).*?gy=(?P<gy>[-0-9.eE]+).*?gz=(?P<gz>[-0-9.eE]+)"
)
GAIN_RE = re.compile(r"BALANCER: started.*?\(Kp=(?P<kp>[-0-9.eE]+) Ki=(?P<ki>[-0-9.eE]+) Kd=(?P<kd>[-0-9.eE]+)\)")
FILTER_RE = re.compile(r"FUSION: active filter=(?P<filter>[A-Za-z0-9_]+)|\[Filter: (?P<filter2>[A-Za-z0-9_]+)\]")


def parse_line(line, rows):
    m = DRIVE_RE.search(line)
    if m:
        d = {k: float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['drive'][t].update(d)
        return
    m = SETDRIVE_RE.search(line)
    if m:
        d = {k: float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['setdrive'][t].update(d)
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
    p.add_argument('--timeout', type=int, default=10, help='seconds to wait for connection')
    p.add_argument('--from-log', help='Parse an existing log file')
    p.add_argument('--no-ffill', dest='ffill', action='store_false', help='Disable forward-fill')
    p.set_defaults(ffill=True)
    args = p.parse_args()

    rows = {'drive': defaultdict(dict), 'bal': defaultdict(dict), 'setdrive': defaultdict(dict), 'imu': dict()}
    gain_info = "Unknown Gains"
    filter_info = "Unknown Filter"
    
    if args.from_log:
        print(f"Parsing log file: {args.from_log}")
        with open(args.from_log, 'r', encoding='utf8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                m_gain = GAIN_RE.search(line)
                if m_gain:
                    gain_info = f"Kp={m_gain.group('kp')} Ki={m_gain.group('ki')} Kd={m_gain.group('kd')}"
                m_filter = FILTER_RE.search(line)
                if m_filter:
                    filter_info = m_filter.group('filter')
                parse_line(line, rows)
    else:
        if not args.host:
            print("Error: --host required for live capture.")
            return

        sock = None
        try:
            print(f"Connecting to {args.host}:{args.port}...")
            sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
            sock.settimeout(0.5) # Small timeout to allow KeyboardInterrupt to fire
            
            print("Listening for BALANCER_DBG... (Waiting for 'BALANCER: started' or first DBG line)")
            
            started = False
            line_buf = ""
            recent_lines = [] # Keep a small history to find filter/gain info before "started"
            with open(args.out, 'w', encoding='utf8') as out_fh:
                while True:
                    try:
                        data = sock.recv(4096).decode('utf8', errors='ignore')
                    except socket.timeout:
                        continue 
                    
                    if not data:
                        break
                    line_buf += data
                    while '\n' in line_buf:
                        line, line_buf = line_buf.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        
                        out_fh.write(line + '\n')
                        out_fh.flush()

                        recent_lines.append(line)
                        if len(recent_lines) > 100: recent_lines.pop(0)

                        # Look for gains and filter in any line received
                        m_gain = GAIN_RE.search(line)
                        if m_gain:
                            gain_info = f"Kp={m_gain.group('kp')} Ki={m_gain.group('ki')} Kd={m_gain.group('kd')}"
                            print(f">>> Detected Gains: {gain_info}")

                        m_filter = FILTER_RE.search(line)
                        if m_filter:
                            filter_info = m_filter.group('filter') or m_filter.group('filter2')
                            print(f">>> Detected Filter: {filter_info}")

                        if not started:
                            if "BALANCER: started" in line or "BALANCER_DBG" in line:
                                print(">>> Balancer START detected! Catching up on history...")
                                # Re-parse recent history to catch filter info if missed
                                for prev_line in recent_lines:
                                    m_f = FILTER_RE.search(prev_line)
                                    if m_f: filter_info = m_f.group('filter') or m_f.group('filter2')
                                    m_g = GAIN_RE.search(prev_line)
                                    if m_g: gain_info = f"Kp={m_g.group('kp')} Ki={m_g.group('ki')} Kd={m_g.group('kd')}"
                                started = True
                        
                        if started:
                            parse_line(line, rows)
                            if "BALANCER: stopped" in line:
                                print(">>> Balancer STOP detected. Ending capture.")
                                raise StopIteration
        except (KeyboardInterrupt, StopIteration):
            print("\nCapture finished.")
        finally:
            if sock: sock.close()

    # Build DataFrame
    keys = set(rows['drive'].keys()) | set(rows['bal'].keys()) | set(rows['setdrive'].keys()) | set(rows['imu'].keys())
    if not keys:
        print("No balancer data found.")
        return
    
    times = sorted(keys)
    recs = []
    for t in times:
        r = {'time_ms': t}
        r.update(rows['imu'].get(t, {}))
        r.update(rows['drive'].get(t, {}))
        r.update(rows['bal'].get(t, {}))
        r.update(rows['setdrive'].get(t, {}))
        recs.append(r)
    
    df = pd.DataFrame(recs).sort_values('time_ms')
    df.to_csv(args.csv, index=False)
    print(f"Wrote CSV: {args.csv}")

    # Plot
    try:
        import matplotlib.pyplot as plt
        expected = ['pitch', 'pitch_setpoint', 'pid_out', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
        for c in expected:
            if c not in df.columns: df[c] = np.nan
        
        for c in df.columns:
            if c != 'time_ms':
                df[c] = pd.to_numeric(df[c], errors='coerce')

        if args.ffill:
            df = df.fillna(method='ffill', limit=10)

        t_axis = (df['time_ms'] - df['time_ms'].iloc[0]) / 1000.0
        
        # Calculate Gyro Pitch (simple integration for lag comparison)
        if 'gy' in df.columns and len(df) > 1:
            dt = df['time_ms'].diff() / 1000.0
            
            # Estimate gyro bias from the first 5 samples where the robot is usually still
            gyro_bias_est = df['gy'].head(5).mean()
            if pd.isna(gyro_bias_est): gyro_bias_est = 0.0
            
            # Use the first valid pitch or 0 as starting point for integration
            start_pitch = df['pitch'].ffill().bfill().iloc[0] if not df['pitch'].dropna().empty else 0.0
            gyro_pitch = [start_pitch]
            for i in range(1, len(df)):
                gy_val = df['gy'].iloc[i]
                dt_val = dt.iloc[i]
                if pd.isna(gy_val) or pd.isna(dt_val):
                    gyro_pitch.append(gyro_pitch[-1])
                else:
                    # Subtract estimated bias and integrate
                    # Note: BNO055 Y is inverted relative to robot pitch in this mapping
                    val = gyro_pitch[-1] - (gy_val - gyro_bias_est) * dt_val
                    gyro_pitch.append(val)
            df['gyro_pitch_est'] = gyro_pitch

        plt.figure(figsize=(12, 12))
        
        # Subplot 1: Pitch
        ax1 = plt.subplot(3, 1, 1)
        ax1.plot(t_axis, df['pitch_setpoint'], 'r--', label='Setpoint', alpha=0.7)
        ax1.plot(t_axis, df['pitch'], 'b-', label='Filtered Pitch (deg)', linewidth=2)
        if 'gyro_pitch_est' in df.columns:
            ax1.plot(t_axis, df['gyro_pitch_est'], 'c--', label='Gyro Integration (No-Lag Ref)', alpha=0.6)
        
        # Draw a horizontal line for the trim if it was detected in the logs
        m_trim = re.search(r"using calibrated trim = ([-0-9.]+) deg", "".join(open(args.out).readlines()) if not args.from_log else "")
        if m_trim:
             trim_val = float(m_trim.group(1))
             ax1.axhline(y=trim_val, color='gray', linestyle=':', label=f'Trim ({trim_val} deg)', alpha=0.5)

        ax1.set_ylabel('Degrees')
        ax1.legend()
        ax1.grid(True)
        ax1.set_title(f'Balancer Pitch stability ({filter_info} | {gain_info})')

        # Subplot 2: PID Output / Command
        ax2 = plt.subplot(3, 1, 2, sharex=ax1)
        if 'pid_out' in df.columns:
            ax2.plot(t_axis, df['pid_out'], 'g-', label='PID Out (Balancer)')
        if 'iterm' in df.columns:
            ax2.plot(t_axis, df['iterm'], 'm:', label='I-Term', alpha=0.5)
        if 'pid_out_drive' in df.columns:
            ax2.plot(t_axis, df['pid_out_drive'], 'r.', label='PID Out (Drive)', alpha=0.5)
        if 'cmd' in df.columns:
            ax2.plot(t_axis, df['cmd'], 'b--', label='Final Cmd', alpha=0.6)
        if 'tgtV' in df.columns:
            ax2.plot(t_axis, df['tgtV'], 'orange', label='Target Velocity', alpha=0.3)
        ax2.set_ylabel('Command')
        ax2.legend()
        ax2.grid(True)

        # Subplot 3: IMU (Accel/Gyro)
        ax3 = plt.subplot(3, 1, 3, sharex=ax1)
        ax3.plot(t_axis, df['gx'], label='Gyro X', alpha=0.5)
        ax3.plot(t_axis, df['gy'], label='Gyro Y', alpha=0.5)
        ax3.plot(t_axis, df['ax'], label='Accel X', alpha=0.5)
        ax3.set_ylabel('IMU Raw')
        ax3.set_xlabel('Time (s)')
        ax3.legend()
        ax3.grid(True)

        plt.tight_layout()
        plt.savefig(args.png)
        print(f"Saved plot: {args.png}")
        
    except Exception as e:
        print(f"Plotting failed: {e}")

if __name__ == '__main__':
    main()

