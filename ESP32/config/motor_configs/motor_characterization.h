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
// Measured: ~40491.743 counts/sec per raw command unit (LEFT)
#define MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD 40491.743f
#endif

#ifndef MOTOR_CHAR_LEFT_GAIN_RPM_PER_CMD
// Equivalent RPM per command unit
#define MOTOR_CHAR_LEFT_GAIN_RPM_PER_CMD 986.000f
#endif

#ifndef MOTOR_CHAR_LEFT_TAU_S
// Time constant (seconds) from telemetry fit (LEFT) - corrected measurement
#define MOTOR_CHAR_LEFT_TAU_S 0.12f
#endif

#ifndef MOTOR_CHAR_LEFT_LATENCY_S
// Measured latency (seconds) for LEFT - realistic measurement ~18ms
#define MOTOR_CHAR_LEFT_LATENCY_S 0.018f
#endif

#ifndef MOTOR_CHAR_LEFT_INVERT
// Invert sign (0 = normal, 1 = invert)
#define MOTOR_CHAR_LEFT_INVERT 0
#endif

#ifndef MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD
// Measured: ~-39469.970 counts/sec per raw command unit (RIGHT)
#define MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD -39469.970f
#endif

#ifndef MOTOR_CHAR_RIGHT_GAIN_RPM_PER_CMD
// Equivalent RPM per command unit
#define MOTOR_CHAR_RIGHT_GAIN_RPM_PER_CMD -961.119f
#endif

#ifndef MOTOR_CHAR_RIGHT_TAU_S
// Time constant (seconds) from telemetry fit (RIGHT) - corrected measurement
#define MOTOR_CHAR_RIGHT_TAU_S 0.12f
#endif

#ifndef MOTOR_CHAR_RIGHT_LATENCY_S
// Measured latency (seconds) for RIGHT - realistic measurement ~18ms
#define MOTOR_CHAR_RIGHT_LATENCY_S 0.018f
#endif

#ifndef MOTOR_CHAR_RIGHT_INVERT
// Invert sign (0 = normal, 1 = invert)
#define MOTOR_CHAR_RIGHT_INVERT 1
#endif

// Time constant (seconds) estimated from step response - corrected value
#ifndef MOTOR_CHAR_TAU_S
#define MOTOR_CHAR_TAU_S 0.12f
#endif

// Measured latency (seconds) - updated with realistic measurement
#ifndef MOTOR_CHAR_LATENCY_S
#define MOTOR_CHAR_LATENCY_S 0.018f
#endif

// Wheel diameter in millimeters
#ifndef MOTOR_CHAR_WHEEL_DIAM_MM
#define MOTOR_CHAR_WHEEL_DIAM_MM 67.0f
#endif

// Deadzone command threshold (command units, e.g., normalized 0..1)
#ifndef MOTOR_CHAR_DEADZONE_CMD
#define MOTOR_CHAR_DEADZONE_CMD 0.02f
#endif

// Invert motor command sign if required (0 = normal, 1 = invert)
#ifndef MOTOR_CHAR_INVERT
#define MOTOR_CHAR_INVERT 0
#endif
