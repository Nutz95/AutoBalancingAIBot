# Implementation Tasks: add-imufusion-warmup-init


1. Design: define warmup API and integration points
   - [x] Draft `ESP32/include/imu_fusion.h` API additions (`beginWarmup`, `isReady`, `getWarmupProgress`, optional `forceReady`).
2. Implement: fusion warmup logic
   - [x] Update `ESP32/src/imu_fusion.cpp` to collect N samples during warmup, compute initial accel-based attitude and gyro bias, and set ready flag.
3. Integrate: IMU task and balancer
   - [x] Call `fusion.beginWarmup(samples)` after IMU initialization completes.
   - [x] Ensure IMU task continues to call `fusion.update(...)` for each sample and that warmup progresses.
   - [x] Update balancer startup (`BALANCE START` path) to require `fusion.isReady()` before enabling motors; add documented user override CLI with safety confirmation.
   - [x] Status LED: `statusLedInit` and `statusLedUpdateFromConsumer` are present and wired to indicate warmup/ready state.
4. Update commands and docs
   - [x] Replace any existing re-init commands so they use the running fusion instance or call `fusion.beginWarmup(...)` rather than creating a separate local instance.
   - [ ] Update `ESP32/README_TUNING.md` and `ESP32/tools/README_TUNING.md` to document warmup behavior and expected sample counts/time.
5. Tests & validation
   - [ ] Add unit test(s) (where feasible) to exercise warmup logic (`ESP32/test/`), mocking IMU samples.
   - [ ] Bench test: perform a TUNING capture showing warmup messages and that `getPitch()` stabilizes after warmup.
6. Finish: openspec docs and bench report
   - [ ] Create a short bench report and add artifacts similar to previous IMU fusion change.

Notes:
- Keep warmup default sample count conservative (e.g., 200 samples) but configurable via `FusionConfig` or compile-time macro.
- The balancer must retain a safe default: motors disabled until all safety checks, including fusion readiness, pass.
