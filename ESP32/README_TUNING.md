# Tuning Telemetry (TUNING) — Quick Start

This short README explains how to use the `TUNING` CSV telemetry mode implemented in firmware.

Purpose
- Provide a clean, machine-readable CSV stream with fused IMU outputs (pitch + pitch rate) and motor commands.

How to enable
- Open a serial console to the board (native USB) at `921600` baud.
- Send the command: `TUNING START` to begin CSV emission. The firmware will print a CSV header first.
- To stop: `TUNING STOP`.

CSV format
- Header (example):
  - `timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,left_cmd,right_cmd`
- Rows: one CSV row per IMU cycle (sample rate derived from BMI088 sampling).

Recommended bench procedure
1. Ensure the robot is securely mounted on a stable fixture where it cannot fall or cause damage.
2. Power the system and confirm BMI088 shows reasonable gravity on one axis.
3. Open serial monitor: `pio device monitor --baud 921600` (or use your terminal/serial app).
4. Send `TUNING START` and capture the output to a file (or copy/paste). Example using `pio device monitor` piping is platform-specific — use your terminal's copy or a serial logger.
5. Analyze the `pitch_deg` column: compute mean and stddev. Expected: stddev < 0.5° when stable and mean within ±2°.

Beta tuning guidance
- The `beta` parameter controls Madgwick responsiveness. Suggested starting range: `0.08`–`0.12`.
- If the estimate is noisy, decrease `beta` slightly; if it lags real motion, increase it.

Post-processing hints
- Use a simple Python snippet to parse CSV and compute statistics (mean/stddev) on `pitch_deg`.

Safety
- Always have a power cutoff and avoid enabling motors when testing stationary IMU fusion.

Files
- `openspec/changes/add-imufusion-madgwick/bench_report.md` — optional location to store bench results.
