#!/usr/bin/env python3
"""
tools/record_blackbox.py

Continuous recording tool for debugging drift and stability issues.
Captures BALANCER_DBG logs and saves them to a CSV file in real-time.
"""

import socket
import sys
import time
import re
import argparse
import csv
import os
import subprocess
from datetime import datetime

# Updated Regex to match the new BALANCER_DBG line with latency
# Format: "BALANCER_DBG t=%lums pitch=%.2fdeg pid_in=%.3fdeg pid_out=%.3f iterm=%.4f cmd=%.3f lat=%luus\n"
BAL_RE = re.compile(
    r"BALANCER_DBG t=(?P<t>\d+)ms pitch=(?P<pitch>[-0-9.]+)deg "
    r"pid_in=(?P<pid_in>[-0-9.]+)deg pid_out=(?P<pid_out>[-0-9.]+) "
    r"iterm=(?P<iterm>[-0-9.]+) cmd=(?P<cmd>[-0-9.]+) lat=(?P<lat>\d+)us"
)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=2333)
    parser.add_argument('--duration', type=float, default=60.0, help="Recording duration in seconds")
    parser.add_argument('output', type=str, nargs='?', default=None, help="Output CSV file path")
    parser.add_argument('--no-plot', action='store_true', help="Skip automatic plotting at the end")
    args = parser.parse_args()

    if not args.output:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output = f"blackbox_{timestamp}.csv"

    print(f"Connecting to {args.host}:{args.port}...")
    sock = socket.create_connection((args.host, args.port), timeout=5)
    f_sock = sock.makefile('r', encoding='utf-8', errors='ignore')
    print("Connected. Enabling logs...")

    # Enable logs if not already enabled
    sock.sendall(b"LOG ENABLE BALANCER\n")
    time.sleep(0.1)

    print(f"Recording to {args.output} for {args.duration}s. Press Ctrl+C to stop early.")
    
    start_time = time.time()
    count = 0
    
    with open(args.output, 'w', newline='') as csvfile:
        fieldnames = ['time_ms', 'pitch_deg', 'pid_in', 'pid_out', 'iterm', 'cmd', 'lat']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        try:
            while time.time() - start_time < args.duration:
                line = f_sock.readline()
                if not line:
                    continue
                
                match = BAL_RE.search(line)
                if match:
                    row = {
                        'time_ms': int(match.group('t')),
                        'pitch_deg': float(match.group('pitch')),
                        'pid_in': float(match.group('pid_in')),
                        'pid_out': float(match.group('pid_out')),
                        'iterm': float(match.group('iterm')),
                        'cmd': float(match.group('cmd')),
                        'lat': int(match.group('lat'))
                    }
                    writer.writerow(row)
                    count += 1
                    if count % 100 == 0:
                        print(f"  Captured {count} points...", end='\r')
        except KeyboardInterrupt:
            print("\nRecording stopped by user.")
        except Exception as e:
            print(f"\nError: {e}")

    print(f"\nCapture finished. Total points: {count}")
    print(f"Data saved to {args.output}")

    if not args.no_plot and count > 0:
        plot_script = os.path.join(os.path.dirname(__file__), "plot_blackbox.py")
        if os.path.exists(plot_script):
            print(f"Launching visualization and saving PNG...")
            subprocess.run([sys.executable, plot_script, args.output])
        else:
            print(f"\nTo visualize the data, run:")
            print(f"python tools/plot_blackbox.py {args.output}")

if __name__ == "__main__":
    main()
