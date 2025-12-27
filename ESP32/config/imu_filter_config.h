// Choose IMU filter implementation at compile-time.
// Define exactly one of the following before building.
// Default runtime selection: COMPLEMENTARY1D
#pragma once

// Available options: IMU_FILTER_MADGWICK, IMU_FILTER_COMPLEMENTARY, IMU_FILTER_KALMAN1D
#ifndef IMU_FILTER_MADGWICK
#define IMU_FILTER_MADGWICK 1
#endif

// Recommended warmup durations (milliseconds) per filter implementation.
// These values are used by the runtime when a filter is selected to request
// an appropriate warmup so the fusion state and gyro bias stabilize before
// enabling the balancer.
#ifndef IMU_FILTER_WARMUP_MS_MADGWICK
#define IMU_FILTER_WARMUP_MS_MADGWICK 4000ul
#endif
#ifndef IMU_FILTER_WARMUP_MS_KALMAN1D
#define IMU_FILTER_WARMUP_MS_KALMAN1D 2000ul
#endif
#ifndef IMU_FILTER_WARMUP_MS_COMPLEMENTARY1D
#define IMU_FILTER_WARMUP_MS_COMPLEMENTARY1D 500ul
#endif

// Threshold for zero-motion detection (rad/s) used for continuous gyro bias update.
// Lower values make the update stricter (robot must be very still).
// 0.02 rad/s ~= 1.15 deg/s
#ifndef IMU_GYRO_STATIONARY_THRESHOLD_RAD_S
#define IMU_GYRO_STATIONARY_THRESHOLD_RAD_S 0.02f
#endif

// Feature flag to enable/disable automatic gyro bias update during operation.
// If enabled (1), the system will continuously update the gyro bias when the robot is stationary
// and periodically save it to NVS.
// If disabled (0), the gyro bias is only set during initial calibration or manual trim.
// Note: Disabling this does NOT prevent loading the last saved bias from NVS at startup.
// Recommended: 0 (Disabled) for balancing robots to prevent learning bad bias during oscillations.
#ifndef IMU_AUTO_GYRO_BIAS_UPDATE
#define IMU_AUTO_GYRO_BIAS_UPDATE 0
#endif
