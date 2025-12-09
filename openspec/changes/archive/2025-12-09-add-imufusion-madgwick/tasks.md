# Implementation Tasks: add-imufusion-madgwick

## 1. Spec & Design
- [x] 1.1 Create `openspec/changes/add-imufusion-madgwick/proposal.md`
- [x] 1.2 Create `openspec/changes/add-imufusion-madgwick/design.md`
- [x] 1.3 Create `openspec/changes/add-imufusion-madgwick/specs/imu-fusion/spec.md`

## 2. API and Implementation (scoped)
- [x] 2.1 Add `ESP32/include/imu_fusion.h` with public API (init, reset, update, getters)
- [x] 2.2 Add `ESP32/src/imu_fusion.cpp` implementing Madgwick (gyro+accel)
- [x] 2.3 Add unit-style test harness (optional) / sample usage in IMU task

## 3. Integration
- [x] 3.1 Integrate `fusion.update` into IMU task loop at 200Hz
- [x] 3.2 Expose `getPitch()` and `getPitchRate()` to balancing controller
- [x] 3.3 Add `TUNING` CSV streaming mode for PID tuning

## 4. Validation & Tuning
- [x] 4.1 Validate that pitch is stable at rest (<±2° error)
- [x] 4.2 Tune `beta` default value (recommend starting 0.08 - 0.12)
- [x] 4.3 Run bench tests and record tuning logs

### Bench Validation Checklist (recommended)

- [x] Prepare hardware: stable mount, known power, emergency cutoff accessible.
- [x] Upload firmware build that includes `CHANNEL_TUNING` CSV emission.
- [x] Open serial monitor at `921600` and send `TUNING START` to begin CSV stream.
- [x] Capture at least 60 seconds of CSV while the robot is stationary; save to `tuning_capture.csv`.
- [x] Verify CSV header and columns: `timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,left_cmd,right_cmd`.
- [x] Compute mean and standard deviation of `pitch_deg` over a 10s window; expect stddev < 0.5° and mean within ±2°.
- [x] If noisy, reduce `beta` (slower response) or increase (faster response) and repeat capture.
- [x] Record chosen `beta`, sample_rate used, and results into a short bench report file `openspec/changes/add-imufusion-madgwick/bench_report.md`.

Notes:
- The code already sets the fusion `sample_rate` from the BMI088 driver sampling configuration; ensure BMI088 is configured correctly before benching.
- Leave magnetometer-dependent tests out of this bench (not implemented in this change).

## 5. Documentation & Archive
- [x] 5.1 Update `ESP32/README_TUNING.md` with tuning procedure
- [x] 5.2 After bench validation, archive change using `openspec archive`


Notes:
- Keep implementation minimal: no magnetometer support in this change. Magnetometer support can be added later as a separate change if heading control is required.
