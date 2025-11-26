# Design: IMU fusion (Madgwick) for robot balancing

## Context

The robot already provides calibrated BMI088 accelerometer and gyroscope readings. The balancing controller requires a stable estimate of pitch and pitch_rate at 100–200 Hz. The solution must be lightweight (ESP32-S3), deterministic, and resilient to transient accelerations during motion.

## Goals / Non-Goals

**Goals**
- Provide quaternion-based orientation estimate (pitch, roll) with low drift
- Run at 200 Hz within existing IMU task timing budget
- Expose pitch and pitch_rate with minimal API and deterministic latency
- Provide TUNING mode (CSV) to help PID tuning

**Non-Goals**
- Magnetometer support (not included in this change)
- High-level motion control (that is a separate change)

## Algorithm choice: Madgwick (gyro + accel)

**Why Madgwick?**
- Low computational cost and well-suited to MCU
- Estimates full quaternion (handles multi-axis coupling)
- Robust for dynamic motion when tuned (beta parameter)

**Alternatives considered**
- Complementary filter: simpler but less robust for multi-axis coupling
- Kalman/Mahony: more complex or similar cost; Madgwick chosen for existing code familiarity and proven embedded use

## API design

`ESP32/include/imu_fusion.h` public API (proposal):

```cpp
namespace abbot {
namespace fusion {

struct FusionConfig { float beta = 0.1f; float sample_rate = 200.0f; };

class Madgwick {
public:
  Madgwick();
  bool begin(const FusionConfig &cfg);
  void reset();
  // gx,gy,gz in rad/s; ax,ay,az raw accel (units are normalized internally); dt in seconds
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
  void getQuaternion(float &w, float &x, float &y, float &z) const;
  float getPitch() const;   // radians
  float getRoll() const;    // radians
  float getPitchRate() const; // rad/s (projected gyro or derivative)
};

} // fusion
} // abbot
```

## Parameters and tuning
- `beta` (Madgwick gain): starting defaults suggested 0.08–0.12 at 200 Hz
- `dt`: integration timestep from IMU tasks (accurate dt is important)
- Accel normalization: ignore accel correction if magnitude below small threshold (faulty reading)

## Data flow and integration
- BMI088Driver produces debiased gyro (deg/s or rad/s) and accel (m/s^2)
- IMU task converts gyro to rad/s and calls `fusion.update(...)` at each tick
- Fusion outputs `getPitch()` and `getPitchRate()` consumed by `balance_controller`

## Tuning & testing
- Provide serial CSV mode: `TUNING ON` → stream timestamp,pitch (deg),pitch_rate (deg/s),left_cmd,right_cmd
- Validate at rest: pitch ≈ expected; small noise band (<±2°)
- Dynamic test: gentle disturbances compensated; tune `beta` for trade-off speed/noise

## Performance
- Floating-point math; measured cost per update ≈ small number of multiplies/adds (< ~2k FLOPs)
- At 200 Hz, CPU load negligible on ESP32-S3 with rest of tasks active

## Risks / Trade-offs
- Without magnetometer yaw will drift – acceptable for balance-only phase
- Aggressive `beta` causes sensitivity to transient accelerations → tune conservatively

## Migration plan
1. Add `imu_fusion` files and API (this change)
2. Hook `fusion.update` into IMU task loop and expose getters
3. Implement `balance_controller` consuming pitch/pitch_rate
4. Tune on bench and enable `TUNING` telemetry

## Open questions
- Should `pitch_rate` be derived from quaternion or taken directly from debiased gyro? (Take gyro for low latency in PID; provide both if useful.)
- Should we add an auto-calibration routine to estimate `beta` on first run? (deferred)

## References
- Madgwick, S.O.H., "An efficient orientation filter for inertial and inertial/magnetic sensor arrays", 2010
- Existing BMI088 driver in `ESP32/src` for reading sensors
