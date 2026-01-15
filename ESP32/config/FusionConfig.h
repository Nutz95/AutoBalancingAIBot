#pragma once
#include <cstdint>

namespace abbot {
namespace fusion {

struct FusionConfig {
    // Madgwick gain: how much to trust accelerometer vs gyroscope
    // Lower value (0.01-0.05) = trust gyro more, smoother but may drift slowly
    // Higher value (0.1-0.3) = trust accel more, corrects drift but noisier
    float beta = 0.05f; 
    float sample_rate = 500.0f; // Hz (Matches BMI160 sampling rate)
    // Axis mapping: allow remapping/inversion of sensor axes into the robot
    // coordinate frame used by the fusion/filter. This makes it easy to
    // accommodate different sensor mountings without changing fusion code.
    //
    // accel_map[i] specifies which sensor axis (0=ax,1=ay,2=az) should be
    // used as robot-axis i (i==0 -> robot X, i==1 -> robot Y, i==2 -> robot Z).
    // accel_sign[i] is either +1 or -1 to flip the sign if needed.
    //
    // BMI088 mounting on this robot (Flipped: Back to sky):
    //   - Robot X = forward (pitch axis rotation) -> Sensor X (Inverted: Forward tilt gives ax < 0)
    //   - Robot Y = right (roll axis rotation)    -> Sensor Y
    //   - Robot Z = up                            -> Sensor Z (Inverted: Standing still gives az < 0)
    //
    // Mapping: robot X := sensor X, robot Y := sensor Y, robot Z := sensor Z
    // Sign adjustment: 
    // - Invert robot X (accel_sign[0]=-1): to get positive pitch for forward tilt (when sensor ax < 0)
    // - Robot Y normal (accel_sign[1]=1): sensor Y orientation matches
    // - Robot Z normal (accel_sign[2]=1): standing az is positive (cfg already inverts it)
    int accel_map[3] = {0, 1, 2};
    int accel_sign[3] = {-1, 1, 1};

    // same for gyro (rad/s) - must match accel mapping for consistent fusion
    int gyro_map[3] = {0, 1, 2};
    int gyro_sign[3] = {-1, 1, 1};
};

} // namespace fusion
} // namespace abbot
