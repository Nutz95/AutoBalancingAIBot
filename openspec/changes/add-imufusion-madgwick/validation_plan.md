# Validation Plan — add-imufusion-madgwick

Purpose: provide step-by-step bench validation and acceptance criteria for the Madgwick fusion change.

Prerequisites
- Hardware: ESP32-S3 board with BMI088 connected, stable bench mount, power supply, optional scope/logger.
- Firmware: latest build with `CHANNEL_TUNING` and `LOG_*` changes deployed.
- Serial: capability to capture serial at `921600` baud.

Steps
1. Build & upload
   - `Push-Location 'i:\GIT\AutoBalancingAIBot\ESP32'; pio run --target upload; Pop-Location`

2. Verify BMI088
   - Open serial monitor and check WHO_AM_I probe messages from BMI088 driver. Confirm accel/gyro IDs and that gravity ~9.81 m/s^2 appears on one axis when tilted.

3. Start TUNING stream
   - Open serial monitor at 921600.
   - Send `TUNING START`.
   - Confirm header line printed and subsequent CSV rows stream at the configured sample rate.

4. Capture stationary data
   - Place robot in a stable, level orientation.
   - Capture 60s of CSV to file `tuning_capture.csv`.

5. Analyze CSV (example Python)
   - Compute mean and stddev of `pitch_deg` over the capture and over a 10s window:
     - stddev(pitch_deg) should be < 0.5° (preferably <0.2°).
     - mean(pitch_deg) should be within ±2° of zero for a level bench.
   - Verify `pitch_rate_deg` is near zero at rest.

6. Dynamic sanity check
   - Apply a slow tilt (~5°) and observe pitch follows smoothly without spikes or NaNs.
   - Apply a step rotation (short manual rotation) and verify estimator remains stable (no blow-up).

7. Beta tuning
   - If estimator is too noisy, reduce `beta` (e.g., from 0.1 → 0.08) and repeat capture.
   - If estimator lags motion, increase `beta` (e.g., 0.1 → 0.12).
   - Record best `beta` and sample_rate in `openspec/changes/add-imufusion-madgwick/bench_report.md`.

8. Acceptance criteria
   - No NaN or infinite values in quaternion or pitch/roll during captures (NaNs = FAIL).
   - Stationary stddev < 0.5° and mean within ±2°.
   - Estimator responds to applied motions without instability.

9. Archive
   - When accepted, add an archive entry in `openspec/changes/archive/` with `tasks.md` updated and bench report attached.

Contact
- If anything fails, attach `tuning_capture.csv` and serial logs and open an issue referencing this change.
