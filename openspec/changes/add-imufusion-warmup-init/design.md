
# Design: IMU fusion warmup and readiness

**Status: [2025-12-08] — Implemented and validated on real hardware.**


Goal
----
Ensure the Madgwick fusion instance is initialized from live IMU samples and signals readiness only when initial attitude and gyro biases are estimated. This prevents starting closed-loop balancing on uninitialized state.

Approach
--------
- Add warmup behavior inside the `fusion` module (not ad-hoc outside):
  - `beginWarmup(int sample_count)` — mark fusion in warmup state and reset internal accumulators.
  - During warmup, `update(gx,gy,gz,ax,ay,az,dt)` still accepts samples but accumulates them for bias/attitude estimation and does not mark the filter fully ready.
  - After `sample_count` IMU samples are collected, compute initial attitude from the averaged accelerometer vector (gravity), compute gyro bias from averaged gyro readings, apply those to the Madgwick internal state, set `ready=true` and continue normal `update` operation.
  - Expose `bool isReady()` and `float getWarmupProgress()` (0..1) so consumers and UI can observe state.

Integration
-----------
- IMU initialization sequence:
  1. Initialize BMI088 driver and confirm data stream.
  2. Call `fusion.beginWarmup(default_samples)` once the IMU is producing valid samples.
  3. The IMU task continues to push samples to `fusion.update(...)`; fusion counts them towards warmup.
- Balancer startup:
  - `BALANCE START` will query `fusion.isReady()` before enabling motors. If not ready, operator is notified (serial log) and the command is deferred or rejected unless the operator uses a documented `BALANCE START FORCE` with explicit confirmation.

Edge cases & safety
-------------------
- Timeout: allow a configurable warmup timeout (e.g., 5 seconds) after which fusion reports failure and requires operator intervention.
- User override: `BALANCE START FORCE` should require an explicit safety confirmation string to reduce accidental enabling.
- Backwards compatibility: keep existing `fusion.reset()` / `init()` semantics but prefer `beginWarmup` flow. Commands that previously reinitialized Madgwick should be updated to use API on the singleton fusion instance.

Configuration
-------------
- Default warmup samples: 200 (≈1.2s at 166–200Hz). Make configurable via `FusionConfig.warmup_samples` or a `FUSION_WARMUP_SAMPLES` macro.

Telemetry & logs
----------------
- During warmup emit periodic serial logs: "FUSION: warmup X/Y samples (progress P%)" and final: "FUSION: warmup complete; gyro_bias=..."
