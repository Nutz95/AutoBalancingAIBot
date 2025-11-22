# Tasks for `swap-bmi088-cs-pins`

1. [x] Update `ESP32/include/BMI088Config.h` defaults for `accel_cs_pin` and `gyro_cs_pin` (swap values).
   - Status: Done. `BMI088Config.h` defaults are set to `accel_cs_pin = 4` and `gyro_cs_pin = 14`.
2. [x] Build the ESP32 project to ensure no compile errors: `pio run`.
   - Status: Done. Project builds and uploads successfully on the ESP32-S3 target.
3. [x] Upload to target board and run quick smoke test:
   - Start serial monitor at the configured baud.
   - Verify accelerometer and gyro outputs look correct (not swapped).
   - Status: Done. Runtime logs show gravity (~9.8 m/s²) appears on the expected axis for tilt tests and WHO_AM_I probes are printed.
4. [x] Update OpenSpec spec delta for BMI088 to note the default pin correction.
   - Status: Done. `openspec/changes/swap-bmi088-cs-pins/specs/bmi088/spec.md` updated with notes about expected IDs and SPI behavior.
5. [x] Run `openspec validate swap-bmi088-cs-pins --strict` and resolve any issues.
   - Status: Done. `openspec validate` returned a successful validation result in the user's environment.
6. [ ] Commit the code and OpenSpec change files.
   - Status: Pending. Files have been edited in the workspace; please review and commit with your normal Git workflow.

7. (Optional) Add an OpenSpec note documenting that the driver includes an SPI dummy-read fix and per-chip SPI mode handling.
   - Status: Done. `openspec/project.md` and `proposal.md` were updated to mention these fixes.
