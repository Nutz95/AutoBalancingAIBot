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
import math
from collections import defaultdict

import numpy as np
import pandas as pd


DRIVE_RE = re.compile(r"DRIVE DBG t=(?P<t>\d+)ms.*?tgtV=(?P<tgtV>[-0-9.eE]+).*?filtV=(?P<filtV>[-0-9.eE]+).*?pitch_setpoint=(?P<pitch_setpoint>[-0-9.eE]+)deg.*?pitch_setpoint_rate=(?P<pitch_setpoint_rate>[-0-9.eE]+)deg/s.*?pid_in=(?P<pid_in_drive>[-0-9.eE]+)deg.*?pid_rate=(?P<pid_rate_drive>[-0-9.eE]+)deg/s.*?pid_out=(?P<pid_out_drive>[-0-9.eE]+)")
SETDRIVE_RE = re.compile(r"SETDRIVE: t=(?P<t>\d+)ms v_req=(?P<v_req>[-0-9.eE]+) w_req=(?P<w_req>[-0-9.eE]+)")
BAL_RE = re.compile(
    r"BALANCER_DBG t=(?P<t>\d+)ms.*?pitch=(?P<pitch>[-0-9.eE]+)deg.*?pid_in=(?P<pid_in>[-0-9.eE]+)deg.*?pid_out=(?P<pid_out>[-0-9.eE]+).*?iterm=(?P<iterm>[-0-9.eE]+).*?cmd=(?P<cmd>[-0-9.eE]+)(?:.*?steer=(?P<steer>[-0-9.eE]+))?.*?lat=(?P<lat>\d+)us.*?ax=(?P<ax>[-0-9.eE]+).*?ay=(?P<ay>[-0-9.eE]+).*?az=(?P<az>[-0-9.eE]+).*?gx=(?P<gx>[-0-9.eE]+).*?gy=(?P<gy>[-0-9.eE]+).*?gz=(?P<gz>[-0-9.eE]+)(?:.*?lp_hz=(?P<lp_hz>[-0-9.eE]+))?(?:.*?encL=(?P<encL>[-0-9.eE]+))?(?:.*?encR=(?P<encR>[-0-9.eE]+))?(?:.*?termA=(?P<lqr_angle>[-0-9.eE]+))?(?:.*?termG=(?P<lqr_gyro>[-0-9.eE]+))?(?:.*?termD=(?P<lqr_dist>[-0-9.eE]+))?(?:.*?termS=(?P<lqr_speed>[-0-9.eE]+))?"
)
GAIN_RE_PID = re.compile(r"BALANCER: started \(PID\) \(Kp=(?P<kp>[-0-9.eE]+) Ki=(?P<ki>[-0-9.eE]+) Kd=(?P<kd>[-0-9.eE]+)\)")
GAIN_RE_LQR = re.compile(r"BALANCER: started \(LQR\) \(Kp=(?P<kp>[-0-9.eE]+) Kg=(?P<kg>[-0-9.eE]+) Kd=(?P<kd>[-0-9.eE]+) Ks=(?P<ks>[-0-9.eE]+)\)")
FILTER_RE = re.compile(r"FUSION: active filter=(?P<filter>[A-Za-z0-9_]+)|\[Filter: (?P<filter2>[A-Za-z0-9_]+)\]")


def safe_float(v):
    if v is None:
        return 0.0
    try:
        return float(v)
    except ValueError:
        return 0.0

def parse_line(line, rows):
    m = DRIVE_RE.search(line)
    if m:
        d = {k: safe_float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['drive'][t].update(d)
        return
    m = SETDRIVE_RE.search(line)
    if m:
        d = {k: safe_float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['setdrive'][t].update(d)
        return
    m = BAL_RE.search(line)
    if m:
        d = {k: safe_float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['bal'][t].update(d)
        return
    # try simple CSV-style imu line (legacy TUNING), detect by commas and numeric start
    if ',' in line:
        parts = [p.strip() for p in line.split(',')]
        try:
            t0 = safe_float(parts[0])
            # guess layout similar to TUNING CSV: timestamp_ms, pitch_deg, pitch_rad, pitch_rate_deg, pitch_rate_rad, ax,ay,az,gx,gy,gz,...
            if len(parts) >= 11:
                t = int(t0)
                ax = safe_float(parts[5])
                ay = safe_float(parts[6])
                az = safe_float(parts[7])
                gx = safe_float(parts[8])
                gy = safe_float(parts[9])
                gz = safe_float(parts[10])
                rows['imu'][t] = {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host')
    p.add_argument('--port', type=int, default=2333)
    p.add_argument('--name', help='Base name for output files (e.g. "capture10" will create .txt, .csv, .png)')
    p.add_argument('-o', '--out', default='balancer_capture.txt')
    p.add_argument('--csv', default='balancer_capture.csv')
    p.add_argument('--png', default='balancer_capture.png')
    p.add_argument('--timeout', type=int, default=10, help='seconds to wait for connection')
    p.add_argument('--from-log', help='Parse an existing log file')
    p.add_argument('--no-ffill', dest='ffill', action='store_false', help='Disable forward-fill')
    p.set_defaults(ffill=True)
    args = p.parse_args()

    if args.name:
        args.out = f"{args.name}.txt"
        args.csv = f"{args.name}.csv"
        args.png = f"{args.name}.png"

    rows = {'drive': defaultdict(dict), 'bal': defaultdict(dict), 'setdrive': defaultdict(dict), 'imu': dict()}
    gain_info = "Unknown Gains"
    gains = {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}
    strategy_mode = 'PID' # Default
    filter_info = "Unknown Filter"
    trim_info = {"val": 0.0, "type": "dynamic", "detected": False}
    
    if args.from_log:
        print(f"Parsing log file: {args.from_log}")
        with open(args.from_log, 'r', encoding='utf8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                m_gain_pid = GAIN_RE_PID.search(line)
                m_gain_lqr = GAIN_RE_LQR.search(line)
                if m_gain_pid:
                    gains = {k: float(m_gain_pid.group(k)) for k in ['kp', 'ki', 'kd']}
                    gain_info = f"PID: Kp={gains['kp']} Ki={gains['ki']} Kd={gains['kd']}"
                    strategy_mode = 'PID'
                elif m_gain_lqr:
                    gains = {k: float(m_gain_lqr.group(k)) for k in ['kp', 'kg', 'kd', 'ks']}
                    gain_info = f"LQR: Kp={gains['kp']} Kg={gains['kg']} Kd={gains['kd']} Ks={gains['ks']}"
                    strategy_mode = 'LQR'
                m_filter = FILTER_RE.search(line)
                if m_filter:
                    filter_info = m_filter.group('filter') or m_filter.group('filter2')
                
                # Detect trim info
                if "using calibrated trim =" in line:
                    m = re.search(r"using calibrated trim = ([-0-9.]+) deg", line)
                    if m:
                        trim_info = {"val": float(m.group(1)), "type": "calibrated", "detected": True}
                elif "dynamic trim captured =" in line:
                    m = re.search(r"dynamic trim captured = ([-0-9.]+) deg", line)
                    if m:
                        trim_info = {"val": float(m.group(1)), "type": "dynamic", "detected": True}

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
                        m_gain_pid = GAIN_RE_PID.search(line)
                        m_gain_lqr = GAIN_RE_LQR.search(line)
                        if m_gain_pid:
                            gains = {k: float(m_gain_pid.group(k)) for k in ['kp', 'ki', 'kd']}
                            gain_info = f"PID: Kp={gains['kp']} Ki={gains['ki']} Kd={gains['kd']}"
                            strategy_mode = 'PID'
                            print(f">>> Detected Strategy: {gain_info}")
                        elif m_gain_lqr:
                            gains = {k: float(m_gain_lqr.group(k)) for k in ['kp', 'kg', 'kd', 'ks']}
                            gain_info = f"LQR: Kp={gains['kp']} Kg={gains['kg']} Kd={gains['kd']} Ks={gains['ks']}"
                            strategy_mode = 'LQR'
                            print(f">>> Detected Strategy: {gain_info}")

                        m_filter = FILTER_RE.search(line)
                        if m_filter:
                            filter_info = m_filter.group('filter') or m_filter.group('filter2')
                            print(f">>> Detected Filter: {filter_info}")

                        # Detect trim info
                        if "using calibrated trim =" in line:
                            m = re.search(r"using calibrated trim = ([-0-9.]+) deg", line)
                            if m:
                                trim_info = {"val": float(m.group(1)), "type": "calibrated", "detected": True}
                                print(f">>> {trim_info['type'].upper()} TRIM detected: {trim_info['val']} deg")
                        elif "dynamic trim captured =" in line:
                            m = re.search(r"dynamic trim captured = ([-0-9.]+) deg", line)
                            if m:
                                trim_info = {"val": float(m.group(1)), "type": "dynamic", "detected": True}
                                print(f">>> {trim_info['type'].upper()} TRIM detected: {trim_info['val']} deg")

                        if not started:
                            if "BALANCER: started" in line or "BALANCER_DBG" in line:
                                print(">>> Balancer START detected! Catching up on history...")
                                # Re-parse recent history to catch metadata if missed
                                for prev_line in recent_lines:
                                    m_f = FILTER_RE.search(prev_line)
                                    if m_f: 
                                        filter_info = m_f.group('filter') or m_f.group('filter2')
                                    
                                    m_g_pid = GAIN_RE_PID.search(prev_line)
                                    m_g_lqr = GAIN_RE_LQR.search(prev_line)
                                    if m_g_pid:
                                        gains = {k: float(m_g_pid.group(k)) for k in ['kp', 'ki', 'kd']}
                                        gain_info = f"PID: Kp={gains['kp']} Ki={gains['ki']} Kd={gains['kd']}"
                                        strategy_mode = 'PID'
                                    elif m_g_lqr:
                                        gains = {k: float(m_g_lqr.group(k)) for k in ['kp', 'kg', 'kd', 'ks']}
                                        gain_info = f"LQR: Kp={gains['kp']} Kg={gains['kg']} Kd={gains['kd']} Ks={gains['ks']}"
                                        strategy_mode = 'LQR'
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
    
    # Auto-detect strategy mode if not explicitly caught from header
    if strategy_mode == 'PID':
        lqr_cols = ['lqr_angle', 'lqr_gyro', 'lqr_dist', 'lqr_speed']
        if all(c in df.columns for c in lqr_cols):
            # If we have these columns and they contain non-zero data, it's LQR
            if df[lqr_cols].any().any():
                strategy_mode = 'LQR'
                if gain_info == "Unknown Gains":
                    gain_info = "LQR (Back-detected from logs)"

    df.to_csv(args.csv, index=False)
    print(f"Wrote CSV: {args.csv}")

    # Extra stats calculation for display
    stats_text = ""
    # Ticks to meters scale: (math.pi * 0.067) / 51200.0 (67mm wheel)
    t2m = (math.pi * 0.067) / 51200.0
    
    try:
        valid_pitch = df['pitch'].dropna()
        if not valid_pitch.empty:
            mean_p = valid_pitch.mean()
            std_p = valid_pitch.std()
            stats_text += f"Pitch: Mean={mean_p:.2f}°, Std={std_p:.2f}°\n"
            
            # Dominant frequency estimation using zero crossings
            pitch_vals = valid_pitch.values
            pitch_centered = pitch_vals - np.mean(pitch_vals)
            crossings = np.where(np.diff(np.sign(pitch_centered)))[0]
            duration = (df['time_ms'].iloc[-1] - df['time_ms'].iloc[0]) / 1000.0
            if duration > 0:
                freq = len(crossings) / (2 * duration)
                stats_text += f"Est. Oscillation Freq: {freq:.2f} Hz\n"

        if 'encL' in df.columns and 'encR' in df.columns:
            start_enc = (df['encL'].iloc[0] + df['encR'].iloc[0]) / 2
            end_enc = (df['encL'].iloc[-1] + df['encR'].iloc[-1]) / 2
            drift_ticks = end_enc - start_enc
            drift_m = drift_ticks * t2m
            stats_text += f"Avg Encoder Drift: {drift_ticks:.1f} ticks ({drift_m:.3f} m)\n"
            
        if strategy_mode == 'LQR' and 'iterm' in df.columns:
            final_trim = df['iterm'].ffill().iloc[-1]
            stats_text += f"Final Adaptive Trim: {final_trim:.3f}°"
    except Exception as e:
        print(f"Stats calculation failed: {e}")

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
            df = df.ffill(limit=10)

        # Calculate PID/LQR components if gains are known
        if strategy_mode == 'PID' and 'pid_in' in df.columns and 'iterm' in df.columns and 'pid_out' in df.columns:
            df['p_term'] = (df['pid_in'] - trim_info['val']) * gains['kp']
            df['d_term'] = df['pid_out'] - df['p_term'] - df['iterm'] # Roughly iterm + d_term
        elif strategy_mode == 'LQR' and 'pitch' in df.columns and 'gy' in df.columns:
            # LQR u = Kp*theta + Kg*gyro - Kd*dist - Ks*speed
            # theta and gyro are in degrees in the log
            # dist is (encL+encR)/2
            # speed is d(dist)/dt
            df['lqr_angle'] = (df['pitch'] - trim_info['val']) * gains['kp']
            df['lqr_gyro'] = df['gy'] * gains['kg']
            if 'encL' in df.columns and 'encR' in df.columns:
                df['dist'] = (df['encL'] + df['encR']) / 2.0
                df['dist_err'] = df['dist'] - df['dist'].iloc[0] # Relative to start of capture
                df['lqr_dist'] = -df['dist_err'] * gains['kd']
                # Velocity estimation (simple)
                dt_s = df['time_ms'].diff() / 1000.0
                df['speed'] = df['dist'].diff() / dt_s
                df['lqr_speed'] = -df['speed'] * gains['ks']

        t_axis = (df['time_ms'] - df['time_ms'].iloc[0]) / 1000.0
        
        # Calculate Gyro Pitch (simple integration for lag comparison)
        if 'gy' in df.columns and len(df) > 1:
            dt_integrated = df['time_ms'].diff() / 1000.0
            
            # Estimate gyro bias from the first 5 samples where the robot is usually still
            gyro_bias_est = df['gy'].head(5).mean()
            if pd.isna(gyro_bias_est): gyro_bias_est = 0.0
            
            # Use the first valid pitch or 0 as starting point for integration
            start_pitch = df['pitch'].ffill().bfill().iloc[0] if not df['pitch'].dropna().empty else 0.0
            gyro_pitch = [start_pitch]
            for i in range(1, len(df)):
                gy_val = df['gy'].iloc[i]
                dt_val = dt_integrated.iloc[i]
                if pd.isna(gy_val) or pd.isna(dt_val):
                    gyro_pitch.append(gyro_pitch[-1])
                else:
                    # Subtract estimated bias and integrate
                    # Note: BNO055 Y is inverted relative to robot pitch in this mapping
                    val = gyro_pitch[-1] - (gy_val - gyro_bias_est) * dt_val
                    gyro_pitch.append(val)
            df['gyro_pitch_est'] = gyro_pitch

        plt.figure(figsize=(12, 24))
        
        # Subplot 1: Pitch
        ax1 = plt.subplot(7, 1, 1)
        ax1.plot(t_axis, df['pitch_setpoint'], 'r--', label='Target (User)', alpha=0.7)
        ax1.plot(t_axis, df['pitch'], 'b-', label='Filtered Pitch (deg)', linewidth=2)
        
        if strategy_mode == 'LQR' and 'iterm' in df.columns:
            ax1.plot(t_axis, df['iterm'], 'm:', label='Dynamic Pitch Trim (Adaptive)', linewidth=2)
            
        if 'gyro_pitch_est' in df.columns:
            ax1.plot(t_axis, df['gyro_pitch_est'], 'c--', label='Gyro Integration (Ref)', alpha=0.4)
        
        # Display stats overlay
        if stats_text:
            ax1.text(0.02, 0.95, stats_text, transform=ax1.transAxes, verticalalignment='top', 
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.8), fontsize=10)

        # Display trim info
        if trim_info['detected']:
             color = 'gray' if trim_info['type'] == 'calibrated' else 'red'
             label = f"Trim ({trim_info['val']} deg - {trim_info['type']})"
             ax1.axhline(y=trim_info['val'], color=color, linestyle=':', label=label, alpha=0.8)

        ax1.set_ylabel('Degrees')
        ax1.legend(loc='upper right')
        ax1.grid(True)
        ax1.set_title(f'Balancer Pitch stability ({filter_info} | {gain_info})')

        # Subplot 2: PID Components (Decomposition)
        ax2 = plt.subplot(7, 1, 2, sharex=ax1)
        if strategy_mode == 'PID':
            if 'p_term' in df.columns:
                ax2.plot(t_axis, df['p_term'], label='P contribution (Kp*err)', alpha=0.8)
            if 'iterm' in df.columns:
                ax2.plot(t_axis, df['iterm'], label='I contribution (Ki*sum)', alpha=0.8)
            if 'd_term' in df.columns:
                ax2.plot(t_axis, df['d_term'], label='D contribution (Kd*rate)', alpha=0.8)
            ax2.set_title('PID Decomposition (Influence of each gain)')
        elif strategy_mode == 'LQR':
            if 'lqr_angle' in df.columns:
                ax2.plot(t_axis, df['lqr_angle'], label='Angle (Kp*theta)', alpha=0.8)
            if 'lqr_gyro' in df.columns:
                ax2.plot(t_axis, df['lqr_gyro'], label='Gyro (Kg*rate)', alpha=0.8)
            if 'lqr_dist' in df.columns:
                ax2.plot(t_axis, df['lqr_dist'], label='Dist (Kd*dist)', alpha=0.8)
            if 'lqr_speed' in df.columns:
                ax2.plot(t_axis, df['lqr_speed'], label='Speed (Ks*vel)', alpha=0.8)
            ax2.set_title('LQR Decomposition (Influence of each gain)')
        
        ax2.set_ylabel('PID/LQR Terms')
        ax2.legend(loc='upper right')
        ax2.grid(True)

        # Subplot 3: Loop Frequency and Latency
        ax3 = plt.subplot(7, 1, 3, sharex=ax1)
        if 'lp_hz' in df.columns:
            ax3.plot(t_axis, df['lp_hz'], 'r-', label='Loop Freq (Hz)', alpha=0.7)
            # Adapt target line to data
            max_hz = df['lp_hz'].max()
            target_hz = 1000.0 if max_hz > 700 else 500.0
            ax3.axhline(y=target_hz, color='k', linestyle=':', alpha=0.3)
            ax3.set_ylim(0, target_hz * 1.2)
        
        ax3_twin = ax3.twinx()
        if 'lat' in df.columns:
            ax3_twin.plot(t_axis, df['lat'] / 1000.0, 'b-', label='Bus Latency (ms)', alpha=0.3)
            # Auto-scale latency axis: at least 0.5 to 2ms
            max_lat_ms = (df['lat'].max() / 1000.0) if not df['lat'].empty else 1.0
            ax3_twin.set_ylim(0.5, max(2.0, max_lat_ms * 1.2))
        
        ax3.set_ylabel('Frequency (Hz)', color='r')
        ax3_twin.set_ylabel('Latency (ms)', color='b')
        ax3.set_title(f'Timing Stability (Target: {int(target_hz)}Hz / {1000.0/target_hz:.1f}ms)')
        ax3.grid(True)

        # Subplot 4: Total PID and Motor Command
        ax4 = plt.subplot(7, 1, 4, sharex=ax1)
        if 'pid_out' in df.columns:
            ax4.plot(t_axis, df['pid_out'], 'g-', label='Total PID/LQR Out', linewidth=1.5)
        if 'cmd' in df.columns:
            ax4.plot(t_axis, df['cmd'], 'k--', label='Throttle Cmd', alpha=0.8)
        if 'steer' in df.columns:
            ax4.plot(t_axis, df['steer'], 'r-', label='Steer Cmd', alpha=0.9)
        ax4.set_ylabel('Command')
        ax4.legend(loc='upper right')
        ax4.grid(True)

        # Subplot 5: Navigation Analysis (Linear Distance & Rotation)
        ax5 = plt.subplot(7, 1, 5, sharex=ax1)
        if 'encL' in df.columns and 'encR' in df.columns:
            linear_pos = (df['encL'] + df['encR']) / 2.0
            rotation = (df['encL'] - df['encR']) / 2.0  # Proxy for heading
            # Center them to start at 0 for easier reading
            linear_pos -= linear_pos.iloc[0]
            rotation -= rotation.iloc[0]
            
            ax5.plot(t_axis, linear_pos, 'k-', label='Linear Position (Avg Ticks)', linewidth=1.5)
            ax5.set_ylabel('Linear (Ticks)', color='k')
            
            # Add secondary scale for meters
            # Ticks to meters scale: (math.pi * 0.067) / 51200.0 (67mm wheel)
            t2m = (math.pi * 0.067) / 51200.0
            ax5_m = ax5.twinx()
            ax5_m.set_ylabel('Linear (Meters)', color='gray')
            # Set the limits of the meter axis proportional to the ticks axis
            y1_min, y1_max = ax5.get_ylim()
            ax5_m.set_ylim(y1_min * t2m, y1_max * t2m)
            ax5_m.tick_params(axis='y', labelcolor='gray')

            ax5_rot = ax5.twinx()
            ax5_rot.spines['right'].set_position(('outward', 60))
            ax5_rot.plot(t_axis, rotation, 'c--', label='Rotation/Yaw (Delta Ticks)', alpha=0.6)
            ax5_rot.set_ylabel('Rotation (Ticks)', color='c')
            ax5_rot.tick_params(axis='y', labelcolor='c')
            
            # Combine legends
            lines, labels = ax5.get_legend_handles_labels()
            lines2, labels2 = ax5_rot.get_legend_handles_labels()
            ax5.legend(lines + lines2, labels + labels2, loc='upper left')
            
        ax5.set_title('Navigation Analysis (Forward/Backward & Yaw Orientation)')
        ax5.grid(True)

        # Subplot 6: Yaw Stability and Heading Hold
        ax6 = plt.subplot(7, 1, 6, sharex=ax1)
        if 'gz' in df.columns:
            ax6.plot(t_axis, np.degrees(df['gz']), 'r-', label='Yaw Rate (deg/s)', alpha=0.8)
        if 'steer' in df.columns:
            ax6_twin = ax6.twinx()
            ax6_twin.plot(t_axis, df['steer'], 'b-', label='Steer Output (Heading Hold)', alpha=0.5)
            ax6_twin.set_ylabel('Steer Output', color='b')
            ax6_twin.set_ylim(-1.0, 1.0)
            
            # Combine legends
            lines, labels = ax6.get_legend_handles_labels()
            lines2, labels2 = ax6_twin.get_legend_handles_labels()
            ax6.legend(lines + lines2, labels + labels2, loc='upper left')
        else:
            ax6.legend(loc='upper left')
            
        ax6.set_ylabel('Rate (deg/s)', color='r')
        ax6.set_title('Yaw Stability and Heading Hold')
        ax6.grid(True)

        # Subplot 7: IMU Raw Pitch-Axis
        ax7 = plt.subplot(7, 1, 7, sharex=ax1)
        ax7.plot(t_axis, df['gx'], label='Gyro X (Roll)', alpha=0.3)
        ax7.plot(t_axis, df['gy'], label='Gyro Y (Pitch)', alpha=0.8)
        ax7.plot(t_axis, df['ax'], label='Accel X (Linear)', alpha=0.6)
        ax7.set_ylabel('IMU Raw')
        ax7.set_xlabel('Time (s)')
        ax7.legend(loc='upper right')
        ax7.grid(True)

        plt.tight_layout()
        plt.savefig(args.png)
        print(f"Saved plot: {args.png}")
        
    except Exception as e:
        print(f"Plotting failed: {e}")

if __name__ == '__main__':
    main()

