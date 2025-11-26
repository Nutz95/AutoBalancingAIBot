# Implementation Tasks: add-imufusion-madgwick

## 1. Spec & Design
- [x] 1.1 Create `openspec/changes/add-imufusion-madgwick/proposal.md`
- [x] 1.2 Create `openspec/changes/add-imufusion-madgwick/design.md`
- [x] 1.3 Create `openspec/changes/add-imufusion-madgwick/specs/imu-fusion/spec.md`

## 2. API and Implementation (scoped)
- [ ] 2.1 Add `ESP32/include/imu_fusion.h` with public API (init, reset, update, getters)
- [ ] 2.2 Add `ESP32/src/imu_fusion.cpp` implementing Madgwick (gyro+accel)
- [ ] 2.3 Add unit-style test harness (optional) / sample usage in IMU task

## 3. Integration
- [ ] 3.1 Integrate `fusion.update` into IMU task loop at 200Hz
- [ ] 3.2 Expose `getPitch()` and `getPitchRate()` to balancing controller
- [ ] 3.3 Add `TUNING` CSV streaming mode for PID tuning

## 4. Validation & Tuning
- [ ] 4.1 Validate that pitch is stable at rest (<±2° error)
- [ ] 4.2 Tune `beta` default value (recommend starting 0.08 - 0.12)
- [ ] 4.3 Run bench tests and record tuning logs

## 5. Documentation & Archive
- [ ] 5.1 Update `ESP32/README_TUNING.md` with tuning procedure
- [ ] 5.2 After bench validation, archive change using `openspec archive`

Notes:
- Keep implementation minimal: no magnetometer support in this change. Magnetometer support can be added later as a separate change if heading control is required.
