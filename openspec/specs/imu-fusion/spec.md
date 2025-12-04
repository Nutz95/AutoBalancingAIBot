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

