#pragma once
#include <cstdint>

namespace abbot {
namespace fusion {

struct FusionConfig {
    float beta = 0.084f;        // Madgwick gain (default tuned from capture)
    float sample_rate = 166.66667f; // Hz (measured sample rate)
    // Axis mapping: allow remapping/inversion of sensor axes into the robot
    // coordinate frame used by the fusion/filter. This makes it easy to
    // accommodate different sensor mountings without changing fusion code.
    //
    // accel_map[i] specifies which sensor axis (0=ax,1=ay,2=az) should be
    // used as robot-axis i (i==0 -> robot X, i==1 -> robot Y, i==2 -> robot Z).
    // accel_sign[i] is either +1 or -1 to flip the sign if needed.
    // Default mapping for your mounting: sensor X+ -> right, Y+ -> forward
    // Map sensor axes into robot axes: robot X := sensor Y, robot Y := sensor X
    int accel_map[3] = {1, 0, 2};
    int accel_sign[3] = {1, 1, 1};

    // same for gyro (rad/s)
    int gyro_map[3] = {1, 0, 2};
    int gyro_sign[3] = {1, 1, 1};
};

} // namespace fusion
} // namespace abbot
