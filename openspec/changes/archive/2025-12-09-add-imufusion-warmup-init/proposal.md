# Change: Ensure IMU fusion (Madgwick) warmup before balancer starts

## Why

Balancing requires a stable estimate of pitch and pitch-rate. The Madgwick
fusion implementation must be initialized and warmed with a short sequence of
IMU samples so that initial orientation (from accelerometer) and gyro bias
estimates converge before closed-loop control begins. If the balancer starts
before the fusion estimator is ready the controller can receive biased or
uninitialized angles and command motors dangerously.

## What Changes

- Add a lightweight warmup/init protocol for `imu_fusion` so it can collect a
  configurable number of IMU samples (gyro+accel) after IMU initialization,
  compute an initial attitude and gyro bias, and expose a readiness flag.
- Ensure the balancer (and other consumers) check `fusion.isReady()` before
  starting motors or closing the control loop. Provide a safe timeout / user
  override for advanced users.
- Update any existing commands or startup paths that currently re-initialize
  Madgwick to instead use the already-initialized fusion instance.

## Impact

- Code changes in `ESP32/include/imu_fusion.h`, `ESP32/src/imu_fusion.cpp`, the
  IMU task integration (where `fusion.update` is called), and the balancer
  startup path in `ESP32/src/balancer_controller_impl.cpp` or equivalent.
- User-visible behavior: when the board boots the fusion will be warmed for
  N samples (configurable, default ~200) giving the user ~1s to place the
  robot upright before `BALANCE START` will actually enable motors.

## Acceptance criteria

- `openspec validate add-imufusion-warmup-init --strict` passes.
- The current firmware (Dec 2025) already implements:
  - The `imu_fusion` warmup/init API (`beginWarmup`, `isReady`, `getWarmupProgress`).
  - The balancer only starts if `fusion.isReady()` is true (or documented CLI override).
  - The status LED reflects warmup/ready state.
- Only documentation and unit tests remain to be completed for formal validation. Bench report is not required.

## LED indicator (visual feedback)

- Rationale: Operators need a clear, visible signal that the IMU fusion is
  warming and when it is ready so they can safely place the robot upright
  before closing the control loop. The board exposes a single RGB LED (a
  single NeoPixel/WS2812) on supported hardware; an uncomplicated wrapper
  API will be provided so firmware code can set the LED color without
  depending on a specific third-party helper implementation.
- Behavior: the system SHALL set the LED to **red** while fusion warmup is
  in progress, and to **green** when `fusion.isReady()` becomes true.
  The LED SHALL be left off by default on boards without a configured LED.
- Deliverable: document and add a minimal `statusLed` API (init, setColor,
  off) and call sites: IMU warmup path will set red during progress and
  green on readiness. Ensure calls are non-blocking and safe from RT loops.
