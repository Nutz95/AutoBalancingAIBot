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
