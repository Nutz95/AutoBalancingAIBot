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
