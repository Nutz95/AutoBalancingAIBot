#pragma once
#include <cstdint>

namespace abbot {
namespace fusion {

struct FusionConfig {
    // Madgwick gain: how much to trust accelerometer vs gyroscope
    // Lower value (0.01-0.05) = trust gyro more, smoother but may drift slowly
    // Higher value (0.1-0.3) = trust accel more, corrects drift but noisier
    // For balancing robot with motor vibrations, use 0.033 (standard Madgwick)
    float beta = 0.021909f;
    float sample_rate = 400.0f; // Hz (target sample rate, synchronized with BMI088Config)
    // Axis mapping: allow remapping/inversion of sensor axes into the robot
    // coordinate frame used by the fusion/filter. This makes it easy to
    // accommodate different sensor mountings without changing fusion code.
    //
    // accel_map[i] specifies which sensor axis (0=ax,1=ay,2=az) should be
    // used as robot-axis i (i==0 -> robot X, i==1 -> robot Y, i==2 -> robot Z).
    // accel_sign[i] is either +1 or -1 to flip the sign if needed.
    //
    // BMI088 mounting on this robot:
    //   - Sensor Y+ points forward (ay > 0 when tilted forward)
    //   - Sensor X+ points right
    //   - Sensor Z+ points up
    //
    // Robot coordinate frame (for Madgwick):
    //   - Robot X = forward (pitch axis rotation)
    //   - Robot Y = right (roll axis rotation)  
    //   - Robot Z = up
    //
    // Mapping: robot X := sensor Y, robot Y := sensor X, robot Z := sensor Z
    // Sign adjustment: 
    // - Invert robot X (accel_sign[0]=-1): sensor Y reads negative when tilted forward,
    //   but we want positive pitch for forward tilt
    // - Robot Y normal (accel_sign[1]=+1): sensor X orientation matches
    // - Robot Z normal (accel_sign[2]=+1): sensor Z reads +9.8 when pointing up (correct)
    int accel_map[3] = {1, 0, 2};
    int accel_sign[3] = {1, 1, 1};  // Invert X for correct pitch sign

    // same for gyro (rad/s) - must match accel mapping for consistent fusion
    int gyro_map[3] = {1, 0, 2};
    int gyro_sign[3] = {1, 1, 1};  // Invert X to match accel
};

} // namespace fusion
} // namespace abbot
