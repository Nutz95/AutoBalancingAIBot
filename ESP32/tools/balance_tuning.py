#!/usr/bin/env python3
"""
Balance tuning tool - captures diagnostics data and plots for PID tuning analysis.

Usage:
    python balance_tuning.py COM7 --duration 10 --plot
"""

import argparse
import serial
import time
import csv
import sys
from pathlib import Path
from datetime import datetime

def find_serial_port():
    """Try to auto-detect ESP32 serial port."""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB' in port.description or 'Serial' in port.description:
            return port.device
    return None

def capture_balance_data(port, duration=10, baudrate=921600):
    """Capture balance diagnostics for specified duration."""
    print(f"Opening {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # Wait for connection
    
    # Enable BALANCER channel (diagnostics)
    print("Enabling BALANCER diagnostics...")
    ser.write(b"LOG ENABLE BALANCER\n")
    time.sleep(0.5)
    ser.flushInput()
    
    # Start balancer
    print("Starting balancer...")
    ser.write(b"BALANCE START\n")
    time.sleep(0.5)
    
    print(f"Capturing data for {duration} seconds...")
    print("Hold the robot upright and let it balance!")
    
    data = []
    start_time = time.time()
    
    while (time.time() - start_time) < duration:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Debug: print all lines to see what we're receiving
                    print(f"DEBUG: {line}")
                    # Parse diagnostics lines: timestamp,pitch_deg=X,pitch_rate_deg=Y,left=Z,right=W
                    # or BALANCER: cmd=X err=Y pitch_deg=Z
                    if ',' in line and 'pitch_deg=' in line:
                        # CSV format from diagnostics
                        parts = line.split(',')
                        ts = int(parts[0])
                        pitch = float(parts[1].split('=')[1])
                        pitch_rate = float(parts[2].split('=')[1])
                        left = float(parts[3].split('=')[1])
                        right = float(parts[4].split('=')[1])
                        data.append({
                            'timestamp_ms': ts,
                            'pitch_deg': pitch,
                            'pitch_rate_deg': pitch_rate,
                            'left_cmd': left,
                            'right_cmd': right,
                            'source': 'diag'
                        })
                    elif line.startswith('BALANCER: cmd='):
                        # BALANCER log line
                        parts = line.split()
                        cmd = float(parts[1].split('=')[1])
                        err = float(parts[2].split('=')[1])
                        pitch = float(parts[3].split('=')[1])
                        data.append({
                            'cmd': cmd,
                            'error': err,
                            'pitch_deg': pitch,
                            'source': 'balancer'
                        })
            except Exception as e:
                pass  # Skip malformed lines
    
    # Stop balancer
    print("\nStopping balancer...")
    ser.write(b"BALANCE STOP\n")
    time.sleep(0.5)
    
    # Disable diagnostics
    ser.write(b"LOG DISABLE BALANCER\n")
    ser.close()
    
    print(f"Captured {len(data)} data points")
    return data

def save_data(data, output_file):
    """Save captured data to CSV."""
    if not data:
        print("No data to save!")
        return
    
    # Separate diag and balancer data
    diag_data = [d for d in data if d.get('source') == 'diag']
    bal_data = [d for d in data if d.get('source') == 'balancer']
    
    # Save diagnostics CSV
    diag_file = output_file.with_name(output_file.stem + '_diag.csv')
    if diag_data:
        with open(diag_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['timestamp_ms', 'pitch_deg', 'pitch_rate_deg', 'left_cmd', 'right_cmd'])
            writer.writeheader()
            for row in diag_data:
                writer.writerow({k: row[k] for k in ['timestamp_ms', 'pitch_deg', 'pitch_rate_deg', 'left_cmd', 'right_cmd']})
        print(f"Saved diagnostics data to {diag_file}")
    
    # Save balancer CSV
    bal_file = output_file.with_name(output_file.stem + '_balancer.csv')
    if bal_data:
        with open(bal_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['cmd', 'error', 'pitch_deg'])
            writer.writeheader()
            for row in bal_data:
                writer.writerow({k: row[k] for k in ['cmd', 'error', 'pitch_deg']})
        print(f"Saved balancer data to {bal_file}")

def plot_data(data, output_file=None):
    """Plot captured balance data."""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib not installed. Install with: pip install matplotlib")
        return
    
    # Separate data sources
    diag_data = [d for d in data if d.get('source') == 'diag']
    bal_data = [d for d in data if d.get('source') == 'balancer']
    
    if not diag_data:
        print("No diagnostics data to plot!")
        return
    
    # Extract time series (normalize to start at 0)
    t0 = diag_data[0]['timestamp_ms']
    times = [(d['timestamp_ms'] - t0) / 1000.0 for d in diag_data]
    pitch = [d['pitch_deg'] for d in diag_data]
    pitch_rate = [d['pitch_rate_deg'] for d in diag_data]
    left_cmd = [d['left_cmd'] for d in diag_data]
    right_cmd = [d['right_cmd'] for d in diag_data]
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # Plot 1: Pitch angle
    axes[0].plot(times, pitch, 'b-', label='Pitch angle', linewidth=1.5)
    axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[0].set_ylabel('Pitch (deg)', fontsize=12)
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Balance Performance Analysis', fontsize=14, fontweight='bold')
    
    # Plot 2: Pitch rate
    axes[1].plot(times, pitch_rate, 'g-', label='Pitch rate', linewidth=1.5)
    axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[1].set_ylabel('Pitch rate (deg/s)', fontsize=12)
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Motor commands
    axes[2].plot(times, left_cmd, 'r-', label='Left motor', linewidth=1.5, alpha=0.7)
    axes[2].plot(times, right_cmd, 'm-', label='Right motor', linewidth=1.5, alpha=0.7)
    axes[2].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[2].set_ylabel('Motor command', fontsize=12)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_file:
        plot_file = output_file.with_suffix('.png')
        plt.savefig(plot_file, dpi=150)
        print(f"Saved plot to {plot_file}")

    # Default to non-blocking show to allow script to exit cleanly;
    # callers can request blocking behaviour via CLI `--block`.
    plt.show(block=False)
    # Give the GUI event loop a moment to render; caller may close window manually.
    plt.pause(0.1)

def main():
    parser = argparse.ArgumentParser(description='Balance tuning data capture and analysis')
    parser.add_argument('port', nargs='?', help='Serial port (e.g., COM7 or /dev/ttyUSB0)')
    parser.add_argument('-d', '--duration', type=int, default=10, help='Capture duration in seconds (default: 10)')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='Serial baudrate (default: 921600)')
    parser.add_argument('-o', '--output', type=str, help='Output file base name (default: auto-generated)')
    parser.add_argument('-p', '--plot', action='store_true', help='Plot data after capture')
    parser.add_argument('--block', action='store_true', help='Block and keep plot window open (default: False)')
    parser.add_argument('--no-save', action='store_true', help='Do not save data to CSV')
    
    args = parser.parse_args()
    
    # Auto-detect port if not provided
    port = args.port
    if not port:
        port = find_serial_port()
        if not port:
            print("Error: Could not auto-detect serial port. Please specify manually.")
            sys.exit(1)
        print(f"Auto-detected port: {port}")
    
    # Generate output filename
    if args.output:
        output_file = Path(args.output)
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = Path(f'balance_capture_{timestamp}.csv')
    
    # Capture data
    try:
        data = capture_balance_data(port, args.duration, args.baudrate)
    except KeyboardInterrupt:
        print("\nCapture interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error during capture: {e}")
        sys.exit(1)
    
    # Save data
    if not args.no_save:
        save_data(data, output_file)
    
    # Plot if requested
    if args.plot:
        # If user requested blocking behaviour, show interactive plot and block until closed
        if args.block:
            import matplotlib.pyplot as plt
            plot_data(data, output_file if not args.no_save else None)
            print("Displaying interactive plot (blocking). Close the window to exit.")
            plt.show()
        else:
            plot_data(data, output_file if not args.no_save else None)

if __name__ == '__main__':
    main()
