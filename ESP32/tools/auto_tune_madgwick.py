#!/usr/bin/env python3
"""
tools/auto_tune_madgwick.py

Offline autotune for the Madgwick filter's `beta` parameter using a captured CSV.
The script simulates a Madgwick AHRS from recorded accel+gyro and scores candidates
by drift (slope) and RMSE to a smoothed accel-derived pitch (if available).

Usage:
  python tools/auto_tune_madgwick.py --csv capture.csv
  python tools/auto_tune_madgwick.py --csv capture.csv --beta 0.01:0.5:25

Outputs (in the same dir as input CSV):
  <base>.autotune_madgwick_coarse.csv
  <base>.autotune_madgwick_refine.csv
  <base>.autotune_madgwick_best.png

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
        accel_pitch_rad = np.arctan2(ax, np.sqrt(ay * ay + az * az))
        return np.degrees(accel_pitch_rad)
    return None


def get_gyro_rad_vec(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = len(df)
    gx = np.zeros(n)
    gy = np.zeros(n)
    gz = np.zeros(n)
    if df.shape[1] > 9:
        try:
            gx = df.iloc[:, 8].astype(float).to_numpy()
            gy = df.iloc[:, 9].astype(float).to_numpy()
            gz = df.iloc[:, 10].astype(float).to_numpy()
        except Exception:
            if df.shape[1] > 4:
                g = df.iloc[:, 4].astype(float).to_numpy()
                gx = np.zeros(n)
                gy = g
                gz = np.zeros(n)
    else:
        if df.shape[1] > 4:
            g = df.iloc[:, 4].astype(float).to_numpy()
            gx = np.zeros(n)
            gy = g
            gz = np.zeros(n)
    med = np.nanmedian(np.abs(gy))
    if med > 1.0:
        gx = np.radians(gx)
        gy = np.radians(gy)
        gz = np.radians(gz)
    return gx, gy, gz


def get_time_s(df: pd.DataFrame) -> np.ndarray:
    ts = df.iloc[:, 0].astype(float).to_numpy()
    return (ts - ts[0]) / 1000.0


def quaternion_from_euler(roll: float, pitch: float, yaw: float = 0.0):
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    q0 = cr * cp * cy + sr * sp * sy
    q1 = sr * cp * cy - cr * sp * sy
    q2 = cr * sp * cy + sr * cp * sy
    q3 = cr * cp * sy - sr * sp * cy
    return np.array([q0, q1, q2, q3], dtype=float)


def quat_normalize(q: np.ndarray):
    n = np.linalg.norm(q)
    if n == 0:
        return q
    return q / n


def madgwick_update(q: np.ndarray, gx: float, gy: float, gz: float,
                    ax: float, ay: float, az: float, beta: float, dt: float) -> np.ndarray:
    if dt <= 0:
        return q

    q0, q1, q2, q3 = q

    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return q
    ax /= norm
    ay /= norm
    az /= norm

    _2q0 = 2.0 * q0
    _2q1 = 2.0 * q1
    _2q2 = 2.0 * q2
    _2q3 = 2.0 * q3
    _4q0 = 4.0 * q0
    _4q1 = 4.0 * q1
    _4q2 = 4.0 * q2
    _8q1 = 8.0 * q1
    _8q2 = 8.0 * q2
    q0q0 = q0 * q0
    q1q1 = q1 * q1
    q2q2 = q2 * q2
    q3q3 = q3 * q3

    f1 = _2q1 * q3 - _2q0 * q2 - ax
    f2 = _2q0 * q1 + _2q2 * q3 - ay
    f3 = 1.0 - _2q1 * q1 - _2q2 * q2 - az

    s0 = -_2q2 * f1 + _2q1 * f2
    s1 = _2q3 * f1 + _2q0 * f2 - _4q1 * f3
    s2 = -_2q0 * f1 + _2q3 * f2 - _4q2 * f3
    s3 = _2q1 * f1 + _2q2 * f2

    s = np.array([s0, s1, s2, s3], dtype=float)
    s_norm = np.linalg.norm(s)
    if s_norm > 0.0:
        s /= s_norm

    qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - beta * s[0]
    qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - beta * s[1]
    qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - beta * s[2]
    qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - beta * s[3]

    q0 += qDot0 * dt
    q1 += qDot1 * dt
    q2 += qDot2 * dt
    q3 += qDot3 * dt

    q = np.array([q0, q1, q2, q3], dtype=float)
    q = quat_normalize(q)
    return q


def quaternion_to_pitch_deg(q: np.ndarray) -> float:
    val = 2.0 * (q[0] * q[2] - q[3] * q[1])
    val = max(-1.0, min(1.0, val))
    return math.degrees(math.asin(val))


def simulate_madgwick(time_s: np.ndarray,
                      ax: np.ndarray, ay: np.ndarray, az: np.ndarray,
                      gx: np.ndarray, gy: np.ndarray, gz: np.ndarray,
                      beta: float) -> np.ndarray:
    n = len(time_s)
    pitches = np.zeros(n)

    if n == 0:
        return pitches

    a0x, a0y, a0z = ax[0], ay[0], az[0]
    norm0 = math.sqrt(a0x * a0x + a0y * a0y + a0z * a0z)
    if norm0 == 0.0:
        q = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        a0x /= norm0
        a0y /= norm0
        a0z /= norm0
        roll0 = math.atan2(a0y, a0z)
        pitch0 = math.atan2(-a0x, math.sqrt(a0y * a0y + a0z * a0z))
        q = quaternion_from_euler(roll0, pitch0, 0.0)

    for i in range(n):
        if i == 0:
            dt = 0.0
        else:
            dt = time_s[i] - time_s[i - 1]
            if dt <= 0:
                dt = 0.0

        q = madgwick_update(q, gx[i], gy[i], gz[i], ax[i], ay[i], az[i], beta, dt)
        pitches[i] = quaternion_to_pitch_deg(q)

    return pitches


def score_result(time_s: np.ndarray, angles_deg: np.ndarray, accel_smoothed_deg: np.ndarray):
    try:
        p = np.polyfit(time_s, angles_deg, 1)
        slope = p[0]
    except Exception:
        slope = float('nan')

    if accel_smoothed_deg is not None:
        rmse = np.sqrt(np.nanmean((angles_deg - accel_smoothed_deg) ** 2))
    else:
        rmse = float('nan')

    return slope, rmse


def grid_search(time_s: np.ndarray,
                accel_smoothed_deg: np.ndarray,
                ax: np.ndarray, ay: np.ndarray, az: np.ndarray,
                gx: np.ndarray, gy: np.ndarray, gz: np.ndarray,
                beta_list, weight_drift: float = 1.0, weight_rmse: float = 1.0):
    rows = []
    for beta in beta_list:
        angles_deg = simulate_madgwick(time_s, ax, ay, az, gx, gy, gz, beta)
        slope, rmse = score_result(time_s, angles_deg, accel_smoothed_deg)
        metric = weight_drift * abs(slope) + weight_rmse * (0.0 if math.isnan(rmse) else rmse)
        rows.append({'beta': beta, 'slope': slope, 'rmse': rmse, 'metric': metric})
    return pd.DataFrame(rows)


def parse_range(spec: str):
    if ':' not in spec:
        v = float(spec)
        return np.array([v])
    a, b, s = spec.split(':')
    start = float(a)
    stop = float(b)
    steps = int(s)
    return np.linspace(start, stop, steps)


def main():
    p = argparse.ArgumentParser(description='Offline autotune Madgwick beta from capture.csv')
    p.add_argument('--csv', required=True)
    p.add_argument('--beta', default='0.01:0.5:25', help='range start:stop:steps or single value')
    p.add_argument('--coarse-only', action='store_true')
    p.add_argument('--refine-steps', type=int, default=12)
    p.add_argument('--weight-drift', type=float, default=1.0)
    p.add_argument('--weight-rmse', type=float, default=1.0)
    args = p.parse_args()

    df = load_capture(args.csv)
    time_s = get_time_s(df)
    accel_pitch_deg = compute_accel_pitch_deg(df)

    if df.shape[1] > 7:
        ax = df.iloc[:, 5].astype(float).to_numpy()
        ay = df.iloc[:, 6].astype(float).to_numpy()
        az = df.iloc[:, 7].astype(float).to_numpy()
    else:
        n = len(df)
        ax = np.zeros(n)
        ay = np.zeros(n)
        az = np.zeros(n)

    gx, gy, gz = get_gyro_rad_vec(df)

    accel_smoothed = None
    if accel_pitch_deg is not None:
        try:
            accel_smoothed = pd.Series(accel_pitch_deg).rolling(window=9, min_periods=1, center=True).mean().to_numpy()
        except Exception:
            accel_smoothed = accel_pitch_deg

    beta_list = parse_range(args.beta)

    print(f'Running coarse grid: {len(beta_list)} betas')
    dfres = grid_search(time_s, accel_smoothed, ax, ay, az, gx, gy, gz, beta_list, args.weight_drift, args.weight_rmse)
    dfres = dfres.sort_values('metric')
    outbase = os.path.splitext(args.csv)[0]
    coarse_csv = outbase + '.autotune_madgwick_coarse.csv'
    dfres.to_csv(coarse_csv, index=False)
    print('Coarse results saved to', coarse_csv)
    best = dfres.iloc[0]
    print('Best coarse candidate:', best.to_dict())

    if args.coarse_only:
        return

    beta_best = float(best['beta'])
    beta_span = max(1e-6, abs(beta_best) * 0.5 + 1e-6)
    beta_refine = np.linspace(max(1e-6, beta_best - beta_span), beta_best + beta_span, args.refine_steps)

    print(f'Refining around beta={beta_best:.6f}')
    dfref = grid_search(time_s, accel_smoothed, ax, ay, az, gx, gy, gz, beta_refine, args.weight_drift, args.weight_rmse)
    dfref = dfref.sort_values('metric')
    refine_csv = outbase + '.autotune_madgwick_refine.csv'
    dfref.to_csv(refine_csv, index=False)
    print('Refine results saved to', refine_csv)
    best_ref = dfref.iloc[0]
    print('Best refined candidate:', best_ref.to_dict())

    try:
        import matplotlib.pyplot as plt

        angles_deg = simulate_madgwick(time_s, ax, ay, az, gx, gy, gz, float(best_ref['beta']))
        plt.figure(figsize=(10, 5))
        if accel_smoothed is not None:
            plt.plot(time_s, accel_smoothed, color='orange', alpha=0.9, label='accel-smoothed')
        plt.plot(time_s, angles_deg, color='blue', label=f"madgwick(beta={best_ref['beta']:.4f})")
        plt.xlabel('time (s)')
        plt.ylabel('pitch (deg)')
        plt.legend()
        outpng = outbase + '.autotune_madgwick_best.png'
        plt.tight_layout()
        plt.savefig(outpng, dpi=150)
        plt.close()
        print('Saved best-candidate plot to', outpng)
    except Exception as e:
        print('Failed to create/save best-candidate plot:', e)


if __name__ == '__main__':
    main()
