#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <cassert>
#include <cstdio>
#include "imu_mapping.h"
#include "../config/FusionConfig.h"

// Provide millis() stub for host builds if not defined
#ifndef ARDUINO
unsigned long millis() { static unsigned long m = 0; return ++m; }
#endif

static void test_identity_mapping() {
    fusion::FusionConfig cfg; // defaults assumed identity
    float raw_g[3] = { 0.1f, -0.2f, 0.3f };
    float raw_a[3] = { 1.0f, 2.0f, 3.0f };
    float bias[3] = { 0.01f, -0.02f, 0.0f };
    float out_g[3];
    float out_a[3];
    abbot::imu_mapping::mapSensorToRobot(cfg, raw_g, raw_a, bias, out_g, out_a);
    // gyro bias subtraction
    assert(fabs(out_g[0] - (raw_g[0] - bias[0])) < 1e-6f);
    assert(fabs(out_g[1] - (raw_g[1] - bias[1])) < 1e-6f);
    assert(fabs(out_g[2] - (raw_g[2] - bias[2])) < 1e-6f);
    // accel unchanged
    assert(fabs(out_a[0] - raw_a[0]) < 1e-6f);
    assert(fabs(out_a[1] - raw_a[1]) < 1e-6f);
    assert(fabs(out_a[2] - raw_a[2]) < 1e-6f);
}

static void test_swapped_signed_mapping() {
    fusion::FusionConfig cfg; // modify mapping
    // swap X and Y, invert Z for accel; swap Y,Z for gyro with sign flips
    cfg.accel_map[0] = 1; cfg.accel_sign[0] = 1;  // robot X <- sensor Y
    cfg.accel_map[1] = 0; cfg.accel_sign[1] = -1; // robot Y <- -sensor X
    cfg.accel_map[2] = 2; cfg.accel_sign[2] = -1; // robot Z <- -sensor Z
    cfg.gyro_map[0] = 0;  cfg.gyro_sign[0] = -1;  // robot X <- -sensor X
    cfg.gyro_map[1] = 2;  cfg.gyro_sign[1] = 1;   // robot Y <- sensor Z
    cfg.gyro_map[2] = 1;  cfg.gyro_sign[2] = -1;  // robot Z <- -sensor Y
    float raw_g[3] = { 0.5f, -0.6f, 0.7f };
    float raw_a[3] = { 4.0f, 5.0f, -6.0f };
    float bias[3] = { 0.05f, -0.06f, 0.07f }; // bias in sensor axes
    float out_g[3];
    float out_a[3];
    abbot::imu_mapping::mapSensorToRobot(cfg, raw_g, raw_a, bias, out_g, out_a);
    // Expected gyro (apply mapping & sign THEN subtract mapped bias)
    float mapped_bias[3] = {
        cfg.gyro_sign[0] * bias[cfg.gyro_map[0]],
        cfg.gyro_sign[1] * bias[cfg.gyro_map[1]],
        cfg.gyro_sign[2] * bias[cfg.gyro_map[2]]
    };
    float expected_g[3] = {
        cfg.gyro_sign[0] * raw_g[cfg.gyro_map[0]] - mapped_bias[0],
        cfg.gyro_sign[1] * raw_g[cfg.gyro_map[1]] - mapped_bias[1],
        cfg.gyro_sign[2] * raw_g[cfg.gyro_map[2]] - mapped_bias[2]
    };
    for (int i=0;i<3;++i) {
        assert(fabs(out_g[i] - expected_g[i]) < 1e-6f);
    }
    float expected_a[3] = {
        cfg.accel_sign[0] * raw_a[cfg.accel_map[0]],
        cfg.accel_sign[1] * raw_a[cfg.accel_map[1]],
        cfg.accel_sign[2] * raw_a[cfg.accel_map[2]]
    };
    for (int i=0;i<3;++i) {
        assert(fabs(out_a[i] - expected_a[i]) < 1e-6f);
    }
}

int main() {
    test_identity_mapping();
    test_swapped_signed_mapping();
    printf("[OK] imu_mapping tests passed.\n");
    return 0;
}
