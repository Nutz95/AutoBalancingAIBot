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

TUNING SUMMARY (firmware)
- Example firmware output (appears after `TUNING STOP` or when auto-capture completes):
  - `TUNING SUMMARY: samples=2000, pitch_mean=0.185635, pitch_std=0.085277, pr_mean=0.025836, pr_std=0.136723`
  - `Suggested beta ~ 0.084`

What the fields mean
- `samples`: number of samples collected.
- `pitch_mean`: mean pitch over the capture (degrees when CSV column is `pitch_deg`).
- `pitch_std`: standard deviation of pitch (degrees) — an indicator of angular noise/drift.
- `pr_mean`: mean pitch-rate (degrees/sec when CSV column is `pitch_rate_deg`).
- `pr_std`: standard deviation of pitch-rate (degrees/sec) — indicates angular velocity noise.
- `Suggested beta`: heuristic suggestion from firmware to seed Madgwick's `beta` gain; treat it as a starting point, not a final tuned value.

Quick interpretation (example values above)
- `pitch_mean ~= 0.19°` — very close to zero; good for a stationary bench test.
- `pitch_std ~= 0.085°` — small angular noise; indicates a stable IMU reading on the bench.
- `pr_std ~= 0.137°/s` — modest angular-rate noise.
- `Suggested beta ~ 0.084` — consistent with the low-noise measurements; reasonable starting point. If your system feels sluggish during motion, slightly increase `beta`. If it jitters/noisy, decrease `beta`.

Recommended tuning workflow
1. Run a capture using the host helper so the serial stream is saved to a file. Preferred helper (from repo):
   - `ESP32/tools/capture_tuning.ps1` (PowerShell wrapper) — it will prefer the Python helper if available and defaults to `TUNING START 2000` when run without arguments.
   - Or run: `python ESP32/tools/capture_tuning.py -p COMX -b 921600 -c "TUNING START 2000"` to capture to a timestamped file.
2. Examine the `TUNING SUMMARY` printed by firmware at the end. Use the `pitch_std` and `pr_std` to assess noise levels and to choose a `beta` seed.
3. Try the suggested beta, then test dynamic motion: small nudges and observe latency vs noise. Adjust in small steps (±10–30%).

Notes and assumptions
- The example assumes the CSV columns indicated earlier (`pitch_deg`, `pitch_rate_deg`). If you are using radians columns, convert appropriately.
- The firmware `Suggested beta` is a heuristic. Final tuning should be done with real-world motion tests (balancing, step responses, disturbance rejection).

FRANÇAIS — Mode d'emploi et interprétation rapide
- Le résumé `TUNING SUMMARY` donne les moyennes et écarts-types calculés pendant la capture (Welford en une passe).
- Dans l'exemple ci‑dessus, la moyenne d'assiette (`pitch_mean`) est proche de 0° et l'écart-type (`pitch_std`) est bas — bon signe pour un test statique sur banc.
- `pr_std` (écart‑type de la vitesse angulaire) indique le bruit en vitesse; une valeur faible signifie que l'estimation du taux est propre.
- La `Suggested beta` fournie par le firmware est un point de départ. Testez-la, puis augmentez si l'estimation traîne derrière les mouvements réels, ou diminuez si elle devient trop bruitée.

Souhaitez-vous que j'ajoute un petit script Python d'analyse (moyenne, std, PSD rapide) qui prend le fichier de capture et propose un balayage de `beta` pour comparaison ?
