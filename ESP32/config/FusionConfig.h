#pragma once
#include <cstdint>

namespace abbot {
namespace fusion {

struct FusionConfig {
    float beta = 0.4f;        // Madgwick gain - increased to 0.4 to reduce gyro drift
    float sample_rate = 166.66667f; // Hz (measured sample rate)
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
    // Sign adjustment: invert robot X (accel_sign[0] = -1) so that pitch > 0 means forward tilt
    int accel_map[3] = {1, 0, 2};
    int accel_sign[3] = {-1, 1, 1};  // Invert X to get correct pitch sign

    // same for gyro (rad/s) - must match accel mapping for consistent fusion
    int gyro_map[3] = {1, 0, 2};
    int gyro_sign[3] = {-1, 1, 1};  // Invert X to match accel
};

} // namespace fusion
} // namespace abbot
