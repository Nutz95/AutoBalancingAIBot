#!/usr/bin/env python3
"""
analyze_tuning_capture.py

Simple analysis helper for TUNING capture files.
- Strips non-CSV log lines
- Parses CSV rows with header: timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,left_cmd,right_cmd
- Computes mean/std for pitch and pitch-rate
- Saves plots: time series, histogram, PSD (Welch)
- Usage: analyze_tuning_capture.py <capture_file>

Dependencies: numpy, matplotlib, scipy
Install: pip install numpy matplotlib scipy
"""
import sys
import os
from pathlib import Path
import re

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.signal import welch
except Exception as e:
    print("Missing Python packages for analysis. Install with: pip install numpy matplotlib scipy")
    raise

CSV_HEADER_REGEX = re.compile(r"^timestamp_ms,\s*pitch_deg,\s*pitch_rad,\s*pitch_rate_deg,\s*pitch_rate_rad,\s*ax,\s*ay,\s*az,\s*gx,\s*gy,\s*gz,\s*temp_C", re.I)
ROW_REGEX = re.compile(r"^\s*\d+,")


def find_capture_file(arg=None):
    if arg:
        p = Path(arg)
        if p.exists():
            return p
        else:
            raise FileNotFoundError(arg)
    # find most recent tuning_capture_*.txt in current dir
    files = sorted(Path('.').glob('tuning_capture_*.txt'), key=os.path.getmtime, reverse=True)
    if not files:
        raise FileNotFoundError('No tuning_capture_*.txt files found in current directory')
    return files[0]


def parse_capture(path):
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    header_idx = None
    for i, L in enumerate(lines):
        if CSV_HEADER_REGEX.search(L):
            header_idx = i
            header = L.strip()
            break
    if header_idx is None:
        # try to find first CSV-like line
        for i, L in enumerate(lines):
            if ROW_REGEX.search(L):
                header_idx = i - 1
                header = 'timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,left_cmd,right_cmd'
                break
    if header_idx is None:
        raise ValueError('No CSV header or rows found in file')

    rows = []
    for L in lines[header_idx+1:]:
        if ROW_REGEX.search(L):
            # split and try to parse first 5 numeric fields
            parts = [p.strip() for p in L.split(',')]
            # expected columns: timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd
            if len(parts) < 12:
                continue
            try:
                ts = float(parts[0])
                pitch_deg = float(parts[1])
                pitch_rad = float(parts[2])
                pr_deg = float(parts[3])
                pr_rad = float(parts[4])
                ax = float(parts[5])
                ay = float(parts[6])
                az = float(parts[7])
                gx = float(parts[8])
                gy = float(parts[9])
                gz = float(parts[10])
                temp_C = float(parts[11])
                # keep core values plus raw and temp for possible later analysis
                rows.append((ts, pitch_deg, pitch_rad, pr_deg, pr_rad, ax, ay, az, gx, gy, gz, temp_C))
            except Exception:
                continue
    if not rows:
        raise ValueError('No valid CSV rows parsed')

    data = np.array(rows)
    return header, data


def analyze_and_plot(path, outdir=None):
    header, data = parse_capture(path)
    ts = data[:,0]
    pitch = data[:,1]
    pr = data[:,3]
    temp = data[:,11]

    # basic stats
    stats = {
        'samples': int(data.shape[0]),
        'pitch_mean_deg': float(np.mean(pitch)),
        'pitch_std_deg': float(np.std(pitch, ddof=1)),
        'pr_mean_deg_s': float(np.mean(pr)),
        'pr_std_deg_s': float(np.std(pr, ddof=1)),
        'temp_mean_C': float(np.mean(temp)),
        'temp_std_C': float(np.std(temp, ddof=1)),
    }

    print('Analysis for', path)
    for k,v in stats.items():
        print(f'{k}: {v}')

    if outdir is None:
        outdir = Path(path).with_suffix('')
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # time axis in seconds relative
    t0 = ts[0]
    t = (ts - t0) / 1000.0

    # Time series plots with temperature overlay (secondary y-axis)
    fig, ax1 = plt.subplots(figsize=(10,4))
    ax1.plot(t, pitch, label='pitch (deg)', color='tab:blue')
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('pitch (deg)')
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.plot(t, temp, label='temp (C)', color='tab:red', alpha=0.8)
    ax2.set_ylabel('temp (C)')

    # combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    p1 = outdir / 'pitch_time.png'
    fig.tight_layout()
    fig.savefig(p1)
    plt.close(fig)

    fig, ax1 = plt.subplots(figsize=(10,4))
    ax1.plot(t, pr, label='pitch_rate (deg/s)', color='orange')
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('pitch_rate (deg/s)')
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.plot(t, temp, label='temp (C)', color='tab:red', alpha=0.8)
    ax2.set_ylabel('temp (C)')

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    p2 = outdir / 'pr_time.png'
    fig.tight_layout()
    fig.savefig(p2)
    plt.close(fig)

    # histograms
    plt.figure(figsize=(6,4))
    plt.hist(pitch, bins=60)
    plt.title('Pitch histogram (deg)')
    plt.grid(True)
    p3 = outdir / 'pitch_hist.png'
    plt.tight_layout()
    plt.savefig(p3)
    plt.close()

    plt.figure(figsize=(6,4))
    plt.hist(pr, bins=60)
    plt.title('Pitch rate histogram (deg/s)')
    plt.grid(True)
    p4 = outdir / 'pr_hist.png'
    plt.tight_layout()
    plt.savefig(p4)
    plt.close()

    # PSD using Welch
    fs = estimate_sampling_rate(ts)
    if fs is None:
        print('Could not estimate sample rate reliably; skipping PSD')
    else:
        f_p, Pxx_p = welch(pitch, fs=fs, nperseg=min(1024, len(pitch)))
        plt.figure(figsize=(8,4))
        plt.semilogy(f_p, Pxx_p)
        plt.title('Pitch PSD (Welch)')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('PSD (deg^2/Hz)')
        plt.grid(True)
        p5 = outdir / 'pitch_psd.png'
        plt.tight_layout()
        plt.savefig(p5)
        plt.close()

        f_pr, Pxx_pr = welch(pr, fs=fs, nperseg=min(1024, len(pr)))
        plt.figure(figsize=(8,4))
        plt.semilogy(f_pr, Pxx_pr)
        plt.title('Pitch-rate PSD (Welch)')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('PSD ((deg/s)^2/Hz)')
        plt.grid(True)
        p6 = outdir / 'pr_psd.png'
        plt.tight_layout()
        plt.savefig(p6)
        plt.close()

    # write simple report
    rpt = outdir / 'summary.txt'
    with open(rpt, 'w') as fh:
        fh.write('Analysis summary for: ' + str(path) + '\n')
        for k,v in stats.items():
            fh.write(f'{k}: {v}\n')
        if fs is not None:
            fh.write(f'estimated_sample_rate_Hz: {fs}\n')
            fh.write('\nColumns: timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd\n')

    print('Plots and summary written to', outdir)


def estimate_sampling_rate(ts_ms):
    # estimate median sample dt
    if len(ts_ms) < 5:
        return None
    dts = np.diff(ts_ms)
    # remove zeros and negatives
    dts = dts[dts > 0]
    if len(dts) < 3:
        return None
    median_dt_ms = np.median(dts)
    if median_dt_ms <= 0:
        return None
    return 1000.0 / median_dt_ms


if __name__ == '__main__':
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        f = find_capture_file(arg)
    except Exception as e:
        print('Error locating capture file:', e)
        sys.exit(1)
    try:
        analyze_and_plot(f)
    except Exception as e:
        print('Error analyzing file:', e)
        sys.exit(2)
    sys.exit(0)
