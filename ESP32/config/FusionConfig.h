#pragma once
#include <cstdint>

namespace abbot {
namespace fusion {

struct FusionConfig {
    float beta = 0.084f;        // Madgwick gain (default tuned from capture)
    float sample_rate = 166.66667f; // Hz (measured sample rate)
};

} // namespace fusion
} // namespace abbot
