# Tasks: add-motor-driver-scservo

1. Create `ESP32/include/motor_driver.h` with the public API and configuration constants.
2. Create `ESP32/src/motor_driver.cpp` providing safe stubs: motors disabled by default, logs on enable/disable/setCommand, and `readEncoder()` returning 0.
3. Add `ESP32/config/motor_config.h` (or similar) to hold motor IDs and inversion flags (left=8, right=7 by default).
4. Wire the new driver into `ESP32/TODO_BALANCER.md` and `ESP32/README.md` with usage notes and safety warnings.
5. Replace stubs with `SCServo` integration: implement transport, initialization, set/position commands and encoder reads.
6. Add simple bench tests (manual test instructions + smoke tests) and a `CALIB`-style serial command to toggle motors for bench verification.
7. Validate the change with `openspec validate add-motor-driver-scservo --strict` and iterate until clean.
8. Archive the change after successful bench validation and documentation updates.
