# Tasks: add-imu-calibration-simple

1. Implement the `imu_calibration` module (gyro bias + accel single-position)
   - Files: `ESP32/src/imu_calibration.cpp`, `ESP32/include/imu_calibration.h`
2. Integrate `load_calibration()` at startup and `apply_calibration()` into `BMI088Driver`.
3. Implement minimal serial UI: `CALIB START GYRO`, `CALIB START ACCEL`, `CALIB DUMP`, `CALIB RESET`.
4. Suppress verbose serial debug during calibration (keep only progress/errors).
5. Add safety: refuse calibration if motors are enabled, or automatically disable motors during calibration.
6. Add stability checks (variance/std) while sampling and require stability before accepting samples.
7. Add manual and smoke tests (verify NVS storage and reload at boot).
8. Document the procedure in `ESP32/README_TUNING.md` and update `openspec/project.md`.
9. Validate with `openspec validate` and prepare the change for archive.

## Completion notes

- [x] Implement the `imu_calibration` module — implemented in `ESP32/src/imu_calibration.cpp` and header added.
- [x] Integrate load/apply calibration — `main.cpp` loads and installs calibration; `BMI088Driver::read()` calls `applyCalibrationToSample()`.
- [x] Serial UI implemented — `CALIB START GYRO`, `CALIB START ACCEL`, `CALIB DUMP`, `CALIB RESET` handled by `serialTaskEntry()`.
- [x] Verbose serial suppression — IMU task suppresses debug while `isCalibrating()` is true; calibration prints only progress/errors.
- [x] Safety: motor disable — calibration routines disable motors during sampling via `motor_control` shim (or refuse if enabled).
- [x] Stability checks — sample stddev checked and calibration aborts on instability.
- [x] Manual/smoke tests — user verified persistence and reload; `CALIB DUMP` matches boot-loaded values.
- [x] Documentation — `ESP32/README.md` updated with calibration procedure and defaults.
- [x] Validation — `openspec validate add-imu-calibration-simple --strict` passed; change ready for archive.
