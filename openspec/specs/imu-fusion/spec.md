# imu-fusion Specification

## Purpose
TBD - created by archiving change add-imufusion-madgwick. Update Purpose after archive.
## Requirements
### Requirement: IMU Fusion Estimator
The system SHALL provide an IMU fusion estimator that combines gyroscope and accelerometer data using a Madgwick-style algorithm to produce a quaternion-based orientation estimate and derived Euler angles (pitch, roll).

#### Scenario: Estimator at rest
- **WHEN** IMU is stationary and fused at 200 Hz
- **THEN** pitch and roll SHALL remain stable within ±2° of expected values
- **AND** noise SHALL be within acceptable bounds for PID control

#### Scenario: Estimator under gentle motion
- **WHEN** IMU is moved gently (< 0.5 g transients)
- **THEN** estimator SHALL follow motion without large overshoot
- **AND** estimator SHALL not drift significantly

### Requirement: Public API
The system SHALL expose a C++ API with functions to initialize, reset, update the estimator, and obtain quaternion/pitch/roll/pitch_rate values.

#### Scenario: Calling update
- **WHEN** caller invokes `update(gx,gy,gz,ax,ay,az,dt)` with gx/gy/gz in rad/s and dt in seconds
- **THEN** estimator SHALL update its internal quaternion state
- **AND** subsequent `getPitch()` SHALL return the updated pitch

### Requirement: Update Frequency
The system SHALL support running the estimator at 100–500 Hz. The recommended default is 200 Hz.

#### Scenario: Running at 200 Hz
- **WHEN** update called with dt=0.005
- **THEN** estimator SHALL produce stable quaternion outputs suitable for a 200 Hz control loop

### Requirement: Safety for invalid accel readings
The system SHALL ignore accelerometer-based correction when accel norm is near zero (sensor fault) to avoid corrupting attitude estimate.

#### Scenario: Invalid accel
- **WHEN** accel magnitude < 0.1 (units after read)
- **THEN** estimator SHALL skip accel correction for that update and rely solely on gyro integration

### Requirement: Tuning interface
The system SHALL provide a parameter `beta` to tune the fusion gain (default suggested 0.08–0.12 at 200 Hz).

#### Scenario: Changing beta
- **WHEN** beta is set to a higher value
- **THEN** estimator SHALL correct drift faster but be more sensitive to accel transients

### Requirement: Tuning telemetry
The system SHALL provide an optional CSV/TUNING telemetry mode streaming timestamp,pitch,pitch_rate,motor_cmd fields for PID tuning.

#### Scenario: Tuning enabled
- **WHEN** user enables `TUNING` mode over serial
- **THEN** system SHALL stream CSV lines at reasonable rate (<= 200 Hz) containing values for offline analysis

### Requirement: No magnetometer required
The system SHALL operate without a magnetometer; yaw may drift but pitch/roll SHALL be stable for balancing.

#### Scenario: No magnetometer present
- **WHEN** no magnetometer is attached
- **THEN** estimator SHALL still provide pitch and roll suitable for balance control

### Requirement: Integration contract
The system SHALL expose `getPitch()` and `getPitchRate()` for consumer modules (e.g., `balance_controller`) and guarantee deterministic latency from `update()` to getters.

#### Scenario: Consumer reads pitch immediately
- **WHEN** a consumer calls `getPitch()` immediately after `update()` returns
- **THEN** the returned value SHALL correspond to the latest update

### Requirement: Warmup initialization

- The fusion module SHALL provide a warmup initialization mode that collects a configurable number of IMU samples (gyro + accel) after IMU startup and before the fusion is considered "ready" for control.

#### Scenario: Warmup progress

- Given the IMU is initialized and streaming, when `fusion.beginWarmup(200)` is called, then `fusion.getWarmupProgress()` returns a value that increases from 0 to 1 as samples arrive and `fusion.isReady()` becomes true when the target sample count is reached.

#### Scenario: Initial attitude and gyro bias

- Given `fusion.beginWarmup(N)` ran and N samples were collected, then the fusion module computes an initial attitude from the averaged accelerometer vector and a gyro bias from averaged gyro readings, applies them to the internal filter state, and `fusion.isReady()` returns true.

### Requirement: Consumer readiness check

- Consumers of fusion outputs (e.g., the balancer) SHALL check `fusion.isReady()` and defer enabling motors or closing control loops until readiness is signalled. A documented user override is allowed but must require explicit confirmation.

#### Scenario: Balancer safety

- Given the operator issues `BALANCE START` while `fusion.isReady()` is false, then the system shall refuse to enable motors and log a clear message instructing the operator to wait for warmup or use `BALANCE START FORCE <CONFIRM>` to override.

### Requirement: Configurable warmup parameters

- The fusion module SHALL allow configuring the warmup sample count (default 200) and an optional warmup timeout (default 5s) via `FusionConfig` or compile-time macros.

#### Scenario: Warmup timeout

- Given the IMU stops producing samples during warmup, when the warmup timeout elapses, then `fusion` sets an error state, `fusion.isReady()` remains false, and the system logs an error requiring operator intervention.

### Requirement: Status LED indicator

- The firmware SHALL expose a minimal `statusLed` API that abstracts the
	platform's single RGB LED (NeoPixel/WS2812) and provides the following
	non-blocking operations:

	- `void statusLedInit();` — initialize LED hardware if present.
	- `void statusLedSetColor(uint8_t r, uint8_t g, uint8_t b);` — set LED color.
	- `void statusLedOff();` — turn the LED off.

- Behavior: During the fusion warmup sequence the IMU/ fusion task SHALL call
	`statusLedSetColor(255,0,0)` to indicate **red** (warmup). When
	`fusion.isReady()` becomes true the task SHALL call
	`statusLedSetColor(0,255,0)` to indicate **green** (ready). If no LED is
	configured for the board, the API calls SHALL be no-ops.

#### Implementation notes / example

- A thin wrapper that reuses an existing NeoPixel helper is acceptable — for
	example the Waveshare `RGB_CTRL.h` demonstrates the required operations
	(init + set color). The wrapper SHOULD live under `ESP32/include/status_led.h`
	and provide the API signatures above so higher-level code does not depend
	on the full helper implementation.

- Example usage in the IMU warmup flow (pseudocode):

```cpp
// after IMU initialization
statusLedInit();
fusion.beginWarmup(DEFAULT_WARMUP_SAMPLES);

while(!fusion.isReady()){
	// called from IMU task as samples arrive
	statusLedSetColor(255,0,0); // red while warming
}

// when ready
statusLedSetColor(0,255,0); // green when ready
```

- The LED operations MUST be safe to call from the IMU task or other
	non-blocking contexts. Implementations MUST avoid long delays (no-blocking
	APIs only) and should prefer single-pixel updates.

