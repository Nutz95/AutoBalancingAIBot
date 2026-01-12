#!/usr/bin/env python3
"""
tools/plot_blackbox.py

Visualization tool for blackbox CSV files.
Plots pitch, PID components, and motor commands over time.
"""

import pandas as pd
import matplotlib.pyplot as plt
import argparse
import sys
import os

def main():
    parser = argparse.ArgumentParser(description="Plot blackbox telemetry data.")
    parser.add_argument('csv_file', help="Path to the blackbox CSV file")
    parser.add_argument('--output', help="Path to save the plot (optional)")
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print(f"Error: File {args.csv_file} not found.")
        sys.exit(1)

    try:
        df = pd.read_csv(args.csv_file)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    if df.empty:
        print("Error: CSV file is empty.")
        sys.exit(1)

    # Normalize time to start at 0
    df['time_s'] = (df['time_ms'] - df['time_ms'].iloc[0]) / 1000.0

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

    # Top Plot: Pitch and Setpoint (pid_in is error, so setpoint = pitch - error)
    ax1.plot(df['time_s'], df['pitch_deg'], label='Pitch (deg)', color='blue', linewidth=1.5)
    # If we have pid_in, we can infer setpoint
    if 'pid_in' in df.columns:
        setpoint = df['pitch_deg'] - df['pid_in']
        ax1.plot(df['time_s'], setpoint, '--', label='Setpoint (deg)', color='red', alpha=0.7)
    
    ax1.set_ylabel('Angle (deg)')
    ax1.set_title(f"Blackbox Analysis: {os.path.basename(args.csv_file)}")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Bottom Plot: PID Output, I-Term, and Final Command
    if 'pid_out' in df.columns:
        ax2.plot(df['time_s'], df['pid_out'], label='PID Out (raw)', color='green', alpha=0.6)
    if 'iterm' in df.columns:
        ax2.plot(df['time_s'], df['iterm'], label='I-Term contribution', color='orange', linewidth=2)
    if 'cmd' in df.columns:
        ax2.plot(df['time_s'], df['cmd'], label='Final Cmd (slew+db)', color='black', linewidth=1)

    ax2.set_ylabel('Normalized Output (-1..1)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # Third Plot: RS485 Bus Latency
    if 'lat' in df.columns:
        ax3.plot(df['time_s'], df['lat'], label='Bus Latency (us)', color='purple')
        # Reference line for 400Hz (2500us)
        ax3.axhline(y=2500, color='red', linestyle='--', alpha=0.5, label='400Hz Budget')
        ax3.set_ylabel('Latency (us)')
        ax3.set_title('RS485 Communication Latency (1 motor transaction)')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
    else:
        ax3.text(0.5, 0.5, 'Latency data not available', ha='center', va='center')

    ax3.set_xlabel('Time (s)')

    plt.tight_layout()

    # Determine save path (either provided or derived from CSV name)
    save_path = args.output
    if not save_path:
        save_path = os.path.splitext(args.csv_file)[0] + ".png"

    try:
        plt.savefig(save_path)
        print(f"Plot saved to {save_path}")
    except Exception as e:
        print(f"Failed to save plot: {e}")

    # If output wasn't explicitly requested, show the window too
    if not args.output:
        print("Showing plot. Close window to exit.")
        plt.show()

if __name__ == "__main__":
    main()
