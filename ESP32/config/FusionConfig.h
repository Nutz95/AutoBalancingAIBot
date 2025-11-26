#pragma once
#include <cstdint>

namespace abbot {
namespace fusion {

struct FusionConfig {
    float beta = 0.1f;        // Madgwick gain
    float sample_rate = 200.0f; // Hz
};

} // namespace fusion
} // namespace abbot
