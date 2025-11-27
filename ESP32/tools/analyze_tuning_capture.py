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
    from scipy.signal import detrend as scipy_detrend
    from scipy import stats as scipy_stats
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

    # diagnostics: running mean/std to inspect startup transient
    def running_stats(x, window=200):
        # simple running mean/std (past window)
        import numpy as _np
        if len(x) < window:
            return _np.cumsum(x)/(_np.arange(len(x))+1), _np.zeros_like(x)
        csum = _np.cumsum(x)
        csum2 = _np.cumsum(x*x)
        mean = _np.empty_like(x)
        std = _np.empty_like(x)
        for i in range(len(x)):
            start = max(0, i-window+1)
            n = i-start+1
            s = csum[i] - (csum[start-1] if start>0 else 0.0)
            s2 = csum2[i] - (csum2[start-1] if start>0 else 0.0)
            mean[i] = s / n
            var = (s2 - (s*s)/n) / (n-1) if n>1 else 0.0
            std[i] = _np.sqrt(var) if var>0 else 0.0
        return mean, std

    run_mean_pitch, run_std_pitch = running_stats(pitch, window=200)

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

    # --- Accel-derived pitch vs fused pitch overlay ---
    # Compute simple accel-derived pitch (degrees): pitch_accel = atan2(-ax, sqrt(ay^2+az^2))
    ax_vals = data[:,5]
    ay_vals = data[:,6]
    az_vals = data[:,7]
    pitch_accel = np.degrees(np.arctan2(-ax_vals, np.sqrt(ay_vals*ay_vals + az_vals*az_vals)))

    fig, ax = plt.subplots(figsize=(10,4))
    ax.plot(t, pitch, label='fused pitch (deg)', color='tab:blue', linewidth=1)
    ax.plot(t, pitch_accel, label='accel-derived pitch (deg)', color='tab:orange', alpha=0.8, linewidth=1)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('pitch (deg)')
    ax.set_title('Fused vs accel-derived pitch')
    ax.grid(True)
    ax.legend()
    p_overlay = outdir / 'pitch_accel_vs_fused.png'
    fig.tight_layout()
    fig.savefig(p_overlay)
    plt.close(fig)

    # compute correlation and RMSE between accel-derived and fused pitch
    try:
        corr_acc_fused = float(np.corrcoef(pitch_accel, pitch)[0,1])
        rmse = float(np.sqrt(np.mean((pitch_accel - pitch)**2)))
    except Exception:
        corr_acc_fused = None
        rmse = None
    print('Accel vs fused: corr=', corr_acc_fused, 'rmse=', rmse)

    # plot running mean/std to inspect startup transient
    fig, ax = plt.subplots(figsize=(10,4))
    ax.plot(t, run_mean_pitch, label='running mean (pitch)', color='tab:green')
    ax.fill_between(t, run_mean_pitch - run_std_pitch, run_mean_pitch + run_std_pitch, color='tab:green', alpha=0.2, label='running std')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('pitch (deg)')
    ax.set_title('Running mean/std (window=200 samples)')
    ax.grid(True)
    ax.legend()
    p_rm = outdir / 'pitch_running_stats.png'
    fig.tight_layout()
    fig.savefig(p_rm)
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

    # detrend pitch and plot
    try:
        pitch_detr = scipy_detrend(pitch)
    except Exception:
        pitch_detr = pitch - np.poly1d(np.polyfit(t, pitch, 1))(t)

    fig, ax = plt.subplots(figsize=(10,4))
    ax.plot(t, pitch_detr, label='detrended pitch (deg)', color='tab:purple')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('pitch (deg)')
    ax.set_title('Detrended pitch')
    ax.grid(True)
    ax.legend()
    p_dt = outdir / 'pitch_detrended.png'
    fig.tight_layout()
    fig.savefig(p_dt)
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

    # --- Allan variance for gyro (pitch rate) and for pitch ---
    def allan_variance(x, fs, max_num=50):
        # non-overlapping Allan var for taus doubling
        import numpy as _np
        if fs is None or len(x) < 4:
            return None, None
        dt = 1.0 / fs
        n = len(x)
        max_m = int(_np.floor(n/2))
        taus = []
        adevs = []
        m = 1
        while m <= max_m and len(taus) < max_num:
            # cluster length m samples -> tau = m*dt
            k = int(_np.floor(n / m))
            if k < 2:
                break
            # average over clusters
            y = _np.array([_np.mean(x[i*m:(i+1)*m]) for i in range(k)])
            # Allan variance from adjacent cluster differences
            diff = y[1:] - y[:-1]
            avar = 0.5 * _np.mean(diff*diff)
            adev = _np.sqrt(avar)
            taus.append(m*dt)
            adevs.append(adev)
            m = m * 2
        return _np.array(taus), _np.array(adevs)

    fs = estimate_sampling_rate(ts)
    if fs is not None:
        taus_p, adevs_p = allan_variance(pitch, fs)
        taus_pr, adevs_pr = allan_variance(pr, fs)
        # plot Allan
        if taus_p is not None:
            plt.figure(figsize=(6,4))
            plt.loglog(taus_p, adevs_p, marker='o')
            plt.xlabel('tau (s)')
            plt.ylabel('Allan deviation (deg)')
            plt.grid(True, which='both')
            plt.title('Allan deviation - pitch')
            plt.tight_layout()
            plt.savefig(outdir / 'allan_pitch.png')
            plt.close()
        if taus_pr is not None:
            plt.figure(figsize=(6,4))
            plt.loglog(taus_pr, adevs_pr, marker='o')
            plt.xlabel('tau (s)')
            plt.ylabel('Allan deviation (deg/s)')
            plt.grid(True, which='both')
            plt.title('Allan deviation - pitch rate')
            plt.tight_layout()
            plt.savefig(outdir / 'allan_pr.png')
            plt.close()

    # Temperature correlation analysis
    try:
        # Pearson correlation between temp and pitch / pitch rate
        corr_pitch = float(scipy_stats.pearsonr(temp, pitch)[0])
        corr_pr = float(scipy_stats.pearsonr(temp, pr)[0])
    except Exception:
        corr_pitch = None
        corr_pr = None

    # compute low-frequency pitch (moving average) to see bias vs temp
    def moving_average(x, w=500):
        import numpy as _np
        if w <= 1:
            return x
        c = _np.convolve(x, _np.ones(w), 'same') / w
        return c

    pitch_low = moving_average(pitch, w=max(3, int(fs*0.5)) if fs else 200)

    # scatter plot temp vs pitch_low
    plt.figure(figsize=(6,4))
    plt.scatter(temp, pitch_low, s=6, alpha=0.6)
    plt.xlabel('temp (C)')
    plt.ylabel('low-freq pitch (deg)')
    plt.grid(True)
    plt.title(f'Temp vs low-freq pitch (corr={corr_pitch:.3f})' if corr_pitch is not None else 'Temp vs low-freq pitch')
    plt.tight_layout()
    plt.savefig(outdir / 'temp_vs_pitch_low.png')
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

    # append extended metrics (correlations, Allan)
    with open(rpt, 'a') as fh:
        fh.write('\n-- Extended metrics --\n')
        fh.write(f'corr_temp_pitch: {None if "corr_pitch" not in locals() else corr_pitch}\n')
        fh.write(f'corr_temp_pr: {None if "corr_pr" not in locals() else corr_pr}\n')
        fh.write(f'corr_accel_vs_fused_pitch: {corr_acc_fused}\n')
        fh.write(f'rmse_accel_vs_fused_pitch_deg: {rmse}\n')
        if 'fs' in locals() and fs is not None:
            if 'taus_p' in locals() and taus_p is not None:
                fh.write('allan_pitch_taus_s: ' + ','.join([f'{v:.6g}' for v in taus_p]) + '\n')
                fh.write('allan_pitch_adev: ' + ','.join([f'{v:.6g}' for v in adevs_p]) + '\n')
            if 'taus_pr' in locals() and taus_pr is not None:
                fh.write('allan_pr_taus_s: ' + ','.join([f'{v:.6g}' for v in taus_pr]) + '\n')
                fh.write('allan_pr_adev: ' + ','.join([f'{v:.6g}' for v in adevs_pr]) + '\n')

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
