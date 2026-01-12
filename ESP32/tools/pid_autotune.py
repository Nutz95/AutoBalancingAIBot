#!/usr/bin/env python3
"""
tools/pid_autotune.py

Automated PID tuning runner for the AutoBalancingBot.
Varies Kp, Ki, Kd parameters, observes stability, and computes performance metrics.

Usage:
  python tools/pid_autotune.py --host 192.168.1.159 --kp 0.035 --ki 0.036 --kd 0.0005 --step 0.005
"""

import socket
import sys
import time
import re
import argparse
import numpy as np
from typing import List, Dict, Optional, Tuple

# Regex to match the BALANCER_DBG line from src/balancer_controller_impl.cpp
# Format: "BALANCER_DBG t=%lums pitch=%.2fdeg pid_in=%.3fdeg pid_out=%.3f iterm=%.4f cmd=%.3f\n"
BAL_RE = re.compile(
    r"BALANCER_DBG t=(?P<t>\d+)ms pitch=(?P<pitch>[-0-9.]+)deg "
    r"pid_in=(?P<pid_in>[-0-9.]+)deg pid_out=(?P<pid_out>[-0-9.]+) "
    r"iterm=(?P<iterm>[-0-9.]+) cmd=(?P<cmd>[-0-9.]+)"
)

class PIDAutotune:
    def __init__(self, host: str, port: int = 2333):
        self.host = host
        self.port = port
        self.sock = None
        self.file = None
        self.fall_limit_deg = 15.0  # Stop test if robot leans further than this
        
    def connect(self):
        print(f"Connecting to {self.host}:{self.port}...")
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        self.file = self.sock.makefile('r', encoding='utf-8', errors='ignore')
        print("Connected.")

    def send_command(self, cmd: str):
        if not cmd.endswith('\n'):
            cmd += '\n'
        self.sock.sendall(cmd.encode('utf-8'))
        time.sleep(0.05)

    def run_trial(self, kp: float, ki: float, kd: float, duration_s: float = 10.0) -> Dict:
        """Runs a single trial with the given parameters and returns performance metrics."""
        print(f"\n>>> Trial START: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.6f}")
        
        # Prepare robot: Stop if running
        self.send_command("BALANCE STOP")
        self.send_command("LOG DISABLE BALANCER")
        time.sleep(0.5)
        
        # Update gains
        self.send_command(f"BALANCE GAINS {kp:.4f} {ki:.4f} {kd:.6f}")
        self.send_command("LOG ENABLE BALANCER")
        
        print(f"    Hold the robot vertical, then 3s countdown...")
        for i in range(3, 0, -1):
            print(f"    {i}...")
            time.sleep(1.0)
            
        self.send_command("BALANCE START")
        
        # Collect data
        data = []
        trial_start = time.time()
        diverged = False
        
        # Use raw socket with non-blocking reads to avoid 'makefile' timeout issues
        self.sock.setblocking(False)
        line_buffer = ""
        
        while time.time() - trial_start < duration_s:
            try:
                chunk = self.sock.recv(4096).decode('utf-8', errors='ignore')
                if not chunk:
                    time.sleep(0.01)
                    continue
                
                line_buffer += chunk
                while '\n' in line_buffer:
                    line, line_buffer = line_buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                        
                    match = BAL_RE.search(line)
                    if match:
                        t = int(match.group('t'))
                        pitch = float(match.group('pitch'))
                        
                        data.append({
                            't': t,
                            'pitch': pitch
                        })
                        
                        if abs(pitch) > self.fall_limit_deg:
                            print(f"!!! DIVERGENCE DETECTED (pitch={pitch:.1f}deg)")
                            diverged = True
                            break
                if diverged:
                    break
            except BlockingIOError:
                time.sleep(0.01)
                continue
            except Exception as e:
                # Silently ignore occasional timeout errors from recv
                if "timed out" in str(e).lower():
                    time.sleep(0.01)
                    continue
                print(f"    Read error: {e}")
                break
                
        # Stop trial
        self.send_command("BALANCE STOP")
        self.send_command("LOG DISABLE BALANCER")
        
        if not data:
            return {'kp': kp, 'ki': ki, 'kd': kd, 'score': 9999.0, 'success': False, 'reason': 'No data'}
            
        pitches = [d['pitch'] for d in data]
        abs_pitches = [abs(d['pitch']) for d in data]
        
        # metrics
        rms_pitch = np.sqrt(np.mean(np.square(pitches)))
        max_abs_pitch = max(abs_pitches)
        mean_abs_pitch = np.mean(abs_pitches)
        
        # Score calculation: promote stability and low tilt
        # We want to minimize error. Divergence is a huge penalty.
        score = rms_pitch + (max_abs_pitch * 0.2)
        if diverged or len(data) < 50:
            score += 1000.0
            
        print(f"    Trial finished: RMS={rms_pitch:.3f}, MeanAbs={mean_abs_pitch:.3f}, Score={score:.3f}")
        
        return {
            'kp': kp, 'ki': ki, 'kd': kd,
            'success': not diverged and len(data) > 50,
            'score': score,
            'rms_pitch': rms_pitch,
            'max_abs_pitch': max_abs_pitch,
            'mean_abs_pitch': mean_abs_pitch,
            'samples': len(data)
        }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', required=True)
    parser.add_argument('--kp', type=float, default=0.035)
    parser.add_argument('--ki', type=float, default=0.036)
    parser.add_argument('--kd', type=float, default=0.0005)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--step', type=float, default=0.005, help="Step size for Kp variation")
    args = parser.parse_args()
    
    tune = PIDAutotune(args.host)
    try:
        tune.connect()
    except Exception as e:
        print(f"Error connecting: {e}")
        return

    # Baseline trial
    results = []
    baseline = tune.run_trial(args.kp, args.ki, args.kd, args.duration)
    results.append(baseline)
    
    # Simple Local Search: Vary Kp, Ki, Kd slightly
    print("\nStarting parameter sweep...")
    
    # Define search deltas
    search_space = [
        # (kp, ki, kd) multipliers
        (1.0 + args.step/args.kp, 1.0, 1.0),   # Kp +
        (1.0 - args.step/args.kp, 1.0, 1.0),   # Kp -
        (1.0, 1.0 + 0.001/args.ki, 1.0),       # Ki +
        (1.0, 1.0 - 0.001/args.ki, 1.0),       # Ki -
        (1.0, 1.0, 1.2),                        # Kd +
        (1.0, 1.0, 0.8),                        # Kd -
    ]
    
    for (m_p, m_i, m_d) in search_space:
        trial_kp = args.kp * m_p
        trial_ki = args.ki * m_i
        trial_kd = args.kd * m_d
        
        res = tune.run_trial(trial_kp, trial_ki, trial_kd, args.duration)
        results.append(res)
        
    # Sort by score
    sorted_res = sorted(results, key=lambda x: x['score'])
    
    print("\n" + "="*60)
    print("AUTOTUNE RESULTS (Best to Worst)")
    print("="*60)
    for i, res in enumerate(sorted_res):
        star = "*" if res == baseline else " "
        success = "OK" if res['success'] else "FALL"
        print(f"{i+1:2}. {star} score={res['score']:6.3f} [{success}] Kp={res['kp']:.4f}, Ki={res['ki']:.4f}, Kd={res['kd']:.6f}")
        
    print("\nTo apply the best parameters:")
    best = sorted_res[0]
    print(f"  BALANCE GAINS {best['kp']:.4f} {best['ki']:.4f} {best['kd']:.6f}")
    
    # Ensure robot is stopped
    tune.send_command("BALANCE STOP")
    tune.send_command("LOG DISABLE BALANCER")

if __name__ == "__main__":
    main()
