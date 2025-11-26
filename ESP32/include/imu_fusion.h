#pragma once
#include <cstdint>

namespace abbot {
namespace fusion {

struct FusionConfig {
    float beta = 0.1f;        // Madgwick gain
    float sample_rate = 200.0f; // Hz
};

class Madgwick {
public:
    Madgwick();
    explicit Madgwick(const FusionConfig &cfg);
    bool begin(const FusionConfig &cfg);
    void reset();
    // gx,gy,gz in rad/s; ax,ay,az in same units as read (will be normalized internally); dt in seconds
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    void getQuaternion(float &w, float &x, float &y, float &z) const;
    float getPitch() const;    // radians
    float getRoll() const;     // radians
    float getPitchRate() const; // rad/s (derived from change in pitch)

private:
    float q0_, q1_, q2_, q3_;
    float beta_;
    float last_pitch_;
    float last_dt_;
    float last_pitch_rate_;
};

} // namespace fusion
} // namespace abbot
