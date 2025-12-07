#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <cassert>
#include <cstdio>
#include <cmath>
#include "imu_consumer_helpers.h"
#include "imu_fusion.h"
#include "imu_mapping.h"
#include "../config/FusionConfig.h"

#ifndef ARDUINO
unsigned long millis() { static unsigned long m = 0; return ++m; }
#endif

static void test_finalize_warmup_bias_and_seed() {
    abbot::imu_consumer::ConsumerState st;
    const uint32_t N = 100;
    st.warmup_samples_total = N;
    st.warmup_samples_remaining = 0; // finished
    // Populate sums: constant accel pointing +Z, gyro with constant bias
    float accel_z = 9.8f;
    float bias_gx = 0.02f, bias_gy = -0.03f, bias_gz = 0.01f;
    st.warm_ax_sum = 0.0f; // X accel 0
    st.warm_ay_sum = 0.0f; // Y accel 0
    st.warm_az_sum = accel_z * N;
    st.warm_gx_sum = bias_gx * N;
    st.warm_gy_sum = bias_gy * N;
    st.warm_gz_sum = bias_gz * N;

    fusion::FusionConfig cfg; // identity mapping
    cfg.sample_rate = 200.0f; cfg.beta = 0.1f;
    // create Madgwick-based IMUFilter for the test
    extern abbot::IMUFilter* createMadgwickFilter();
    abbot::IMUFilter* f = createMadgwickFilter();
    f->begin(cfg);
    abbot::imu_consumer::finalizeWarmupIfDone(st, *f, cfg);
    delete f;

    assert(st.gyro_bias_initialized);
    assert(fabs(st.gyro_bias[0] - bias_gx) < 1e-6f);
    assert(fabs(st.gyro_bias[1] - bias_gy) < 1e-6f);
    assert(fabs(st.gyro_bias[2] - bias_gz) < 1e-6f);
    // Accumulators reset
    assert(st.warm_ax_sum == 0.0f && st.warm_az_sum == 0.0f);
    // Initial persist candidate set
    assert(st.pending_initial_persist);
}

int main() {
    test_finalize_warmup_bias_and_seed();
    printf("[OK] consumer warmup tests passed.\n");
    return 0;
}
