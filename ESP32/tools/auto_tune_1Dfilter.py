#!/usr/bin/env python3
"""
tools/auto_tune_filter.py

Offline tuner: reuse a captured CSV to simulate the Complementary 1D filter
and search for good `KACC` and `KBIAS` values. The script runs a coarse
grid search followed by an optional fine search around the best candidate.

Usage examples:
  python tools/auto_tune_filter.py --csv capture.csv
  python tools/auto_tune_filter.py --csv capture.csv --kacc 0.01:0.08:8 --kbias 0.005:0.2:8

The script outputs a small CSV with results and prints the best pair.
"""

import argparse
import math
import os
import sys
from typing import Tuple

import numpy as np
import pandas as pd


def load_capture(csv_path: str):
    rows = []
    with open(csv_path, 'r', encoding='utf8', errors='ignore') as fh:
        for line in fh:
            s = line.strip()
            if not s or ',' not in s:
                continue
            if s.startswith('TUNING') or s.startswith('WIFI-CONSOLE') or s.startswith('timestamp_ms'):
                continue
            parts = [t.strip() for t in s.split(',')]
            try:
                float(parts[0])
            except Exception:
                continue
            nums = []
            for p in parts:
                try:
                    nums.append(float(p))
                except Exception:
                    nums.append(np.nan)
            rows.append(nums)

    if not rows:
        raise RuntimeError('No numeric rows found in ' + csv_path)

    maxcols = max(len(r) for r in rows)
    padded = [r + [np.nan] * (maxcols - len(r)) for r in rows]
    df = pd.DataFrame(padded)
    return df


def compute_accel_pitch_deg(df: pd.DataFrame) -> np.ndarray:
    if df.shape[1] > 7:
        ax = df.iloc[:, 5].astype(float).to_numpy()
        ay = df.iloc[:, 6].astype(float).to_numpy()
        az = df.iloc[:, 7].astype(float).to_numpy()
        # use arctan2(ax, sqrt(ay^2+az^2)) consistent with earlier tool
        accel_pitch_rad = np.arctan2(ax, np.sqrt(ay * ay + az * az))
        return np.degrees(accel_pitch_rad)
    return None


def get_gyro_rate_rad(df: pd.DataFrame) -> np.ndarray:
    # prefer column 4 (pitch_rate_rad), fallback to gyro y at col 9
    if df.shape[1] > 4:
        return df.iloc[:, 4].astype(float).to_numpy()
    if df.shape[1] > 9:
        return df.iloc[:, 9].astype(float).to_numpy()
    return np.zeros(len(df))


def get_time_s(df: pd.DataFrame) -> np.ndarray:
    ts = df.iloc[:, 0].astype(float).to_numpy()
    return (ts - ts[0]) / 1000.0


def simulate_complementary(time_s: np.ndarray,
                           accel_pitch_deg: np.ndarray,
                           gyro_rate_rad: np.ndarray,
                           kacc: float,
                           kbias: float,
                           bias_init: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simulate a simple 1D complementary filter with bias estimator.

    angle_est and bias_est are in radians/rad/s internally.
    Update equations (per sample):
      angle += (gyro_rate - bias) * dt
      error = angle - accel_angle
      angle -= kacc * error
      bias += kbias * error * dt

    kacc: proportional correction factor (unitless)
    kbias: bias integrator gain (1/s)
    """
    n = len(time_s)
    angle = 0.0
    bias = bias_init
    angles = np.zeros(n)
    accel_rad = np.radians(accel_pitch_deg) if accel_pitch_deg is not None else np.zeros(n)

    for i in range(n):
        if i == 0:
            dt = 0.0
        else:
            dt = time_s[i] - time_s[i - 1]
            if dt <= 0:
                dt = 0.0

        g = gyro_rate_rad[i]
        angle = angle + (g - bias) * dt
        error = angle - accel_rad[i]
        angle = angle - kacc * error
        bias = bias + kbias * error * dt
        angles[i] = np.degrees(angle)

    return angles, np.array([bias])


def score_result(time_s: np.ndarray, angles_deg: np.ndarray, accel_smoothed_deg: np.ndarray):
    # drift: slope of angle over time (deg/s)
    try:
        p = np.polyfit(time_s, angles_deg, 1)
        slope = p[0]
    except Exception:
        slope = float('nan')

    # rmse to accel_smoothed when available
    if accel_smoothed_deg is not None:
        rmse = np.sqrt(np.nanmean((angles_deg - accel_smoothed_deg) ** 2))
    else:
        rmse = float('nan')

    return slope, rmse


def grid_search(time_s: np.ndarray,
                accel_smoothed_deg: np.ndarray,
                gyro_rate_rad: np.ndarray,
                kacc_list, kbias_list, bias_init: float = 0.0,
                weight_drift: float = 1.0, weight_rmse: float = 1.0):
    rows = []
    for kacc in kacc_list:
        for kbias in kbias_list:
            angles_deg, bias_est = simulate_complementary(time_s, accel_smoothed_deg, gyro_rate_rad, kacc, kbias, bias_init)
            slope, rmse = score_result(time_s, angles_deg, accel_smoothed_deg)
            metric = weight_drift * abs(slope) + weight_rmse * (0.0 if math.isnan(rmse) else rmse)
            rows.append({'kacc': kacc, 'kbias': kbias, 'slope': slope, 'rmse': rmse, 'metric': metric, 'final_bias': bias_est[0]})
    return pd.DataFrame(rows)


def parse_range(spec: str):
    # spec format: start:stop:steps or single value
    if ':' not in spec:
        v = float(spec)
        return np.array([v])
    a, b, s = spec.split(':')
    start = float(a)
    stop = float(b)
    steps = int(s)
    return np.linspace(start, stop, steps)


def main():
    p = argparse.ArgumentParser(description='Offline autotune KACC/KBIAS from capture.csv')
    p.add_argument('--csv', required=True)
    p.add_argument('--kacc', default='0.01:0.06:6', help='range start:stop:steps or single value')
    p.add_argument('--kbias', default='0.005:0.2:8', help='range start:stop:steps or single value')
    p.add_argument('--bias-init', default='0.0', help='initial gyro bias to subtract (rad/s)')
    p.add_argument('--coarse-only', action='store_true')
    p.add_argument('--refine-steps', type=int, default=8)
    p.add_argument('--weight-drift', type=float, default=1.0)
    p.add_argument('--weight-rmse', type=float, default=1.0)
    args = p.parse_args()

    df = load_capture(args.csv)
    time_s = get_time_s(df)
    accel_pitch_deg = compute_accel_pitch_deg(df)
    gyro_rate_rad = get_gyro_rate_rad(df)

    # compute accel-smoothed baseline used for rmse comparison
    accel_smoothed = None
    if accel_pitch_deg is not None:
        try:
            accel_smoothed = pd.Series(accel_pitch_deg).rolling(window=9, min_periods=1, center=True).mean().to_numpy()
        except Exception:
            accel_smoothed = accel_pitch_deg

    kacc_list = parse_range(args.kacc)
    kbias_list = parse_range(args.kbias)
    bias_init = float(args.bias_init)

    print(f'Running coarse grid: {len(kacc_list)} x {len(kbias_list)} = {len(kacc_list) * len(kbias_list)} combos')
    dfres = grid_search(time_s, accel_smoothed, gyro_rate_rad, kacc_list, kbias_list, bias_init, args.weight_drift, args.weight_rmse)
    dfres = dfres.sort_values('metric')
    outbase = os.path.splitext(args.csv)[0]
    coarse_csv = outbase + '.autotune_coarse.csv'
    dfres.to_csv(coarse_csv, index=False)
    print('Coarse results saved to', coarse_csv)
    best = dfres.iloc[0]
    print('Best coarse candidate:', best.to_dict())

    if args.coarse_only:
        return

    # refine around best
    kacc_best = best['kacc']
    kbias_best = best['kbias']
    # create narrower ranges centered on best
    kacc_span = max(1e-4, abs(kacc_best) * 0.5 + 1e-4)
    kbias_span = max(1e-4, abs(kbias_best) * 0.5 + 1e-4)
    kacc_refine = np.linspace(max(0.0, kacc_best - kacc_span), kacc_best + kacc_span, args.refine_steps)
    kbias_refine = np.linspace(max(0.0, kbias_best - kbias_span), kbias_best + kbias_span, args.refine_steps)

    print(f'Refining around kacc={kacc_best:.6f}, kbias={kbias_best:.6f}')
    dfref = grid_search(time_s, accel_smoothed, gyro_rate_rad, kacc_refine, kbias_refine, bias_init, args.weight_drift, args.weight_rmse)
    dfref = dfref.sort_values('metric')
    refine_csv = outbase + '.autotune_refine.csv'
    dfref.to_csv(refine_csv, index=False)
    print('Refine results saved to', refine_csv)
    best_ref = dfref.iloc[0]
    print('Best refined candidate:', best_ref.to_dict())

    # optional: save a plot of best
    try:
        import matplotlib.pyplot as plt

        angles_deg, final_bias = simulate_complementary(time_s, accel_smoothed, gyro_rate_rad, best_ref['kacc'], best_ref['kbias'], bias_init)
        plt.figure(figsize=(10, 5))
        if accel_smoothed is not None:
            plt.plot(time_s, accel_smoothed, color='orange', alpha=0.9, label='accel-smoothed')
        plt.plot(time_s, angles_deg, color='blue', label=f"filter(kacc={best_ref['kacc']:.4f},kbias={best_ref['kbias']:.4f})")
        plt.xlabel('time (s)')
        plt.ylabel('pitch (deg)')
        plt.legend()
        outpng = outbase + '.autotune_best.png'
        plt.tight_layout()
        plt.savefig(outpng, dpi=150)
        plt.close()
        print('Saved best-candidate plot to', outpng)
    except Exception as e:
        print('Failed to create/save best-candidate plot:', e)


if __name__ == '__main__':
    main()
