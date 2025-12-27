// motor_characterization.h - template config based on characterization results
#pragma once

// Ticks per wheel revolution (quadrature counts * gear reduction)
#ifndef MOTOR_CHAR_TICKS_PER_REV
#define MOTOR_CHAR_TICKS_PER_REV 2464
#endif

// Steady-state gain: RPM per command_unit (signed). Positive means positive command
#ifndef MOTOR_CHAR_GAIN_RPM_PER_CMD
#define MOTOR_CHAR_GAIN_RPM_PER_CMD 1000.0f
#endif

// Steady-state gain in counts/sec per command unit (signed)
#ifndef MOTOR_CHAR_GAIN_COUNTS_PER_CMD
#define MOTOR_CHAR_GAIN_COUNTS_PER_CMD (MOTOR_CHAR_GAIN_RPM_PER_CMD * MOTOR_CHAR_TICKS_PER_REV / 60.0f)
#endif

// Per-side measured characterization (prefer these for per-motor tuning)
#ifndef MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD
// Measured: ~42387.65 counts/sec per raw command unit (LEFT) @ 2ms telemetry
#define MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD 31506f
#endif

#ifndef MOTOR_CHAR_LEFT_GAIN_RPM_PER_CMD
// Equivalent RPM per command unit
#define MOTOR_CHAR_LEFT_GAIN_RPM_PER_CMD 1032.5f
#endif

#ifndef MOTOR_CHAR_LEFT_TAU_S
// Time constant (seconds) from telemetry fit (LEFT) - conservative unified value
#define MOTOR_CHAR_LEFT_TAU_S 0.081f
#endif

#ifndef MOTOR_CHAR_LEFT_LATENCY_S
// Measured latency (seconds) for LEFT - conservative max value 20.7ms
#define MOTOR_CHAR_LEFT_LATENCY_S 0.0207f
#endif

#ifndef MOTOR_CHAR_LEFT_INVERT
// Invert sign (0 = normal, 1 = invert)
#define MOTOR_CHAR_LEFT_INVERT 0
#endif

#ifndef MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD
// Measured: ~44102.2 counts/sec per raw command unit (RIGHT) @ 2ms telemetry
#define MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD 38423f
#endif

#ifndef MOTOR_CHAR_RIGHT_GAIN_RPM_PER_CMD
  // Equivalent RPM per command unit
#define MOTOR_CHAR_RIGHT_GAIN_RPM_PER_CMD 1074.3f
#endif

#ifndef MOTOR_CHAR_RIGHT_TAU_S
  // Time constant (seconds) from telemetry fit (RIGHT) - conservative unified value
#define MOTOR_CHAR_RIGHT_TAU_S 0.081f
#endif

#ifndef MOTOR_CHAR_RIGHT_LATENCY_S
  // Measured latency (seconds) for RIGHT - conservative max value 20.7ms
#define MOTOR_CHAR_RIGHT_LATENCY_S 0.0207f
#endif

#ifndef MOTOR_CHAR_RIGHT_INVERT
  // Invert sign (0 = normal, 1 = invert)
#define MOTOR_CHAR_RIGHT_INVERT 1
#endif

  // Time constant (seconds) estimated from step response - conservative unified value (81ms)
#ifndef MOTOR_CHAR_TAU_S
#define MOTOR_CHAR_TAU_S 0.094f
#endif

  // Measured latency (seconds) - conservative max value from 2ms telemetry (20.7ms)
#ifndef MOTOR_CHAR_LATENCY_S
#define MOTOR_CHAR_LATENCY_S 0.0207f
#endif

  // Wheel diameter in millimeters
#ifndef MOTOR_CHAR_WHEEL_DIAM_MM
#define MOTOR_CHAR_WHEEL_DIAM_MM 67.0f
#endif

  // Deadzone command threshold (command units, normalized 0..1)
  // Open interval ]-0.08, 0.08[ - conservative value based on sweep measurements
#ifndef MOTOR_CHAR_DEADZONE_CMD
#define MOTOR_CHAR_DEADZONE_CMD 0.08f
#endif

  // Invert motor command sign if required (0 = normal, 1 = invert)
#ifndef MOTOR_CHAR_INVERT
#define MOTOR_CHAR_INVERT 0
#endif
