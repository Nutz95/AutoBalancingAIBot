## Bench Report: add-motor-driver-scservo

Status: PARTIAL — driver implemented and serial commands available; bench motor tests pending.

Summary of current findings:
- Motor driver module present and compiled into firmware.
- Serial commands available: `MOTOR ENABLE`, `MOTOR DISABLE`, `MOTOR SET`, `MOTOR VEL`, `MOTOR PARAMS`, `MOTOR READ`, `MOTOR DUMP`.
- Default `VELOCITY_MAX_SPEED` currently set in `ESP32/config/motor_config.h`.

Measured / To capture:
- Servo model: STS3215HS (High Speed)
- Observed max `present_speed` at no-load: 39818 steps/s  (Velocity 7000)
- Chosen `VELOCITY_MAX_SPEED` to use in firmware: 7000 (recommend ~90–95% of observed max)
- Default acceleration `acc` used: 0 (recommend 0: max)

Bench steps (safe sequence):
1. Ensure wheels restrained and emergency cutoff ready.
2. Open serial monitor at configured baud and send `MOTOR PARAMS LEFT` / `MOTOR PARAMS RIGHT` to inspect current `acc`, `present_speed`, `torque_limit`.
3. Enable motors: `MOTOR ENABLE` (verify logs show enabled and servos in Wheel mode).
4. Incremental raw velocity test (example):
   - `MOTOR VEL LEFT 0` → `MOTOR PARAMS LEFT` (speed=0)
   - `MOTOR VEL LEFT 500` → `MOTOR PARAMS LEFT` (speed=33268)
   - `MOTOR VEL LEFT 1000` → `MOTOR PARAMS LEFT` (speed=33768)
   - `MOTOR VEL LEFT 4000` → `MOTOR PARAMS LEFT` (speed=36768)
   - `MOTOR VEL LEFT 6000` → `MOTOR PARAMS LEFT` (speed=38718)
   - `MOTOR VEL LEFT 7000` → `MOTOR PARAMS LEFT` (speed=39818)
   - Continue increasing by 500 until `present_speed` no longer increases or approaches servo datasheet limit.
5. Record `present_speed` observed and stop if current/temperature rises significantly.

Safety notes:
- Default firmware sets motors DISABLED on boot; do NOT call `MOTOR ENABLE` unless wheels restrained.
- Stop immediately if servo current or temperature spikes.

Recorder: Nutz95
Date: 2025-12-04
