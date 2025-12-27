// dc_motor_config.h - DC motor / BTS7960 / mirror-mode configuration
// Separated from `servo_motor_config.h` to avoid coupling servo and DC settings.
#pragma once

// =============================================================================
// DC MIRROR MODE CONFIG
// =============================================================================

// Enable the simple mirror fallback driver behaviour (use one working encoder
// to mirror the other motor's encoder when one encoder is broken). Set to 0
// to disable and require both encoders or another driver.
#ifndef DC_MIRROR_MODE_ENABLED
#define DC_MIRROR_MODE_ENABLED 0
#endif

// Specify which encoder(s) are present on the platform. Set 1 if present.
#ifndef DC_ENCODER_PRESENT_LEFT
#define DC_ENCODER_PRESENT_LEFT 1
#endif
#ifndef DC_ENCODER_PRESENT_RIGHT
#define DC_ENCODER_PRESENT_RIGHT 1
#endif

// Which side should be considered the authoritative encoder for mirror mode.
// Options: LEFT_SIDE=0, RIGHT_SIDE=1
#ifndef DC_MIRROR_AUTH_SIDE_RIGHT
#define DC_MIRROR_AUTH_SIDE_RIGHT 1
#endif

// Divergence threshold (encoder counts) beyond which we flag a mismatch and
// optionally disable motors for safety. Tune based on gear ratio and encoder
// resolution; default 1000 counts ≈ ~0.25 turn at 4096 CPR.
#ifndef DC_MIRROR_DIVERGENCE_THRESH
#define DC_MIRROR_DIVERGENCE_THRESH 1000
#endif

// Pins for DC driver (per motor). Each DC motor uses two PWM outputs and two
// enable pins (e.g. for BTS7960). These defaults are placeholders and should
// be adjusted per-board.
#ifndef DC_LEFT_PWM_R_PIN
#define DC_LEFT_PWM_R_PIN 40 // GPIO40 green wire R_PWM
#endif
#ifndef DC_LEFT_PWM_L_PIN
#define DC_LEFT_PWM_L_PIN 39 // GPIO39 blue wire L_PWM
#endif
#ifndef DC_LEFT_EN_R_PIN
#define DC_LEFT_EN_R_PIN 42 // GPIO42 white wire R_En
#endif
#ifndef DC_LEFT_EN_L_PIN
#define DC_LEFT_EN_L_PIN 41 // GPIO41 yellow wire L_En
#endif

#ifndef DC_RIGHT_PWM_R_PIN
#define DC_RIGHT_PWM_R_PIN 17 // GPIO17 green wire R_PWM
#endif
#ifndef DC_RIGHT_PWM_L_PIN
#define DC_RIGHT_PWM_L_PIN 18 // GPIO18 blue wire L_PWM
#endif
#ifndef DC_RIGHT_EN_R_PIN
#define DC_RIGHT_EN_R_PIN 15 // GPIO15 white wire R_En
#endif
#ifndef DC_RIGHT_EN_L_PIN
#define DC_RIGHT_EN_L_PIN 16 // GPIO16 yellow wire L_En
#endif

// -----------------------------------------------------------------------------
// Encoder pin configuration
// Provide two input pins per motor for the quadrature encoder (A and B).
// Set to -1 if not present on this platform.
#ifndef DC_LEFT_ENCODER_A_PIN
#define DC_LEFT_ENCODER_A_PIN 1 // GPIO1 -> Yellow Wire
#endif
#ifndef DC_LEFT_ENCODER_B_PIN
#define DC_LEFT_ENCODER_B_PIN 2 // GPIO2 ->Green Wire
#endif

#ifndef DC_RIGHT_ENCODER_A_PIN
#define DC_RIGHT_ENCODER_A_PIN 6 // GPIO6 -> Yellow Wire
#endif
#ifndef DC_RIGHT_ENCODER_B_PIN
#define DC_RIGHT_ENCODER_B_PIN 7 // GPIO7 -> Green Wire
#endif

// Number of raw signals (pulses) produced on each encoder pin per motor
// revolution. For a typical incremental encoder this is the pulses-per-rev
// per channel. The total quadrature counts per motor revolution is:
//   QUAD_COUNTS = DC_ENCODER_SIGNALS_PER_PIN * 4
// To obtain counts at the output shaft (e.g. wheel) multiply by the
// gear reduction (motor_rev -> wheel_rev):
//   COUNTS_PER_WHEEL_REV = QUAD_COUNTS * DC_ENCODER_GEAR_REDUCTION
#ifndef DC_ENCODER_SIGNALS_PER_PIN
#define DC_ENCODER_SIGNALS_PER_PIN 11
#endif

// Gear reduction ratio (motor revolutions per wheel revolution). Use 1.0f if
// the encoder is mounted on the wheel shaft directly. This can be a float.
#ifndef DC_ENCODER_GEAR_REDUCTION
#define DC_ENCODER_GEAR_REDUCTION 56.0f
#endif

// Convenience macro for quadrature counts per motor revolution (integer).
#ifndef DC_ENCODER_QUADRATURE_COUNTS_PER_MOTOR_REV
#define DC_ENCODER_QUADRATURE_COUNTS_PER_MOTOR_REV (DC_ENCODER_SIGNALS_PER_PIN * 4)
#endif

// Delay (in milliseconds) to wait when performing a safe direction change
// on the H-bridge (deadtime to avoid shoot-through when switching which
// PWM channel is driven). Units: milliseconds.
#ifndef DC_DIRECTION_CHANGE_DELAY_MS
#define DC_DIRECTION_CHANGE_DELAY_MS 1
#endif

// Enable verbose debug logging inside the DC mirror driver (0/1).
// Set to 1 to log direction changes, computed duty, channel selections
// and EN pin actions to the motor log channel.
#ifndef DC_MIRROR_DRIVER_DEBUG
#define DC_MIRROR_DRIVER_DEBUG 1
#endif

// PWM frequency / resolution defaults used by DC driver. Adjust as needed.
#ifndef DC_PWM_FREQUENCY_HZ
// 2Khz is Optimal for BTS7960, It won't support well 20Khz and create a big deadzone.
// 2Khz is also quiet enough for most applications.
// 2Khz will create more torque at low speeds compared to higher frequencies.
#define DC_PWM_FREQUENCY_HZ 4000 
#endif
#ifndef DC_PWM_RESOLUTION_BITS
//14 bits is popssible with ESP32-S3 LEDC @ 2Khz, gives more control resolution than 8 bits.
#define DC_PWM_RESOLUTION_BITS 12
#endif

// DC-specific velocity mapping (separate from servo VELOCITY_* constants)
// Map raw speed units to normalized commands for DC driver simulation and
// for any controllers that expect DC-specific scaling.
#ifndef DC_VELOCITY_MAX_SPEED
#define DC_VELOCITY_MAX_SPEED 7000
#endif

#ifndef DC_VELOCITY_TARGET_INCREMENT_SCALE
#define DC_VELOCITY_TARGET_INCREMENT_SCALE 100.0f
#endif

// DC-specific motor ID / invert macros (if platform uses different IDs
// for DC drivers, override here). These are intentionally DC-prefixed to
// avoid colliding with servo-specific `LEFT_MOTOR_ID` macros.
#ifndef DC_LEFT_MOTOR_ID
#define DC_LEFT_MOTOR_ID 0
#endif
#ifndef DC_RIGHT_MOTOR_ID
#define DC_RIGHT_MOTOR_ID 1
#endif
#ifndef DC_LEFT_MOTOR_INVERT
#define DC_LEFT_MOTOR_INVERT 1
#endif
#ifndef DC_RIGHT_MOTOR_INVERT
#define DC_RIGHT_MOTOR_INVERT 1
#endif

// If a motor characterization header exists, import per-side measured values
#if __has_include("motor_characterization.h")
#include "motor_characterization.h"
#endif

// Prefer per-side characterization macros when available (falls back to global)
#ifdef MOTOR_CHAR_LEFT_INVERT
#undef DC_LEFT_MOTOR_INVERT
#define DC_LEFT_MOTOR_INVERT MOTOR_CHAR_LEFT_INVERT
#endif
#ifdef MOTOR_CHAR_RIGHT_INVERT
#undef DC_RIGHT_MOTOR_INVERT
#define DC_RIGHT_MOTOR_INVERT MOTOR_CHAR_RIGHT_INVERT
#endif

// Expose per-side gain/tau/latency under DC-prefixed macros for firmware use
#ifndef DC_LEFT_GAIN_COUNTS_PER_CMD
#ifdef MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD
#define DC_LEFT_GAIN_COUNTS_PER_CMD MOTOR_CHAR_LEFT_GAIN_COUNTS_PER_CMD
#else
#define DC_LEFT_GAIN_COUNTS_PER_CMD MOTOR_CHAR_GAIN_COUNTS_PER_CMD
#endif
#endif

#ifndef DC_RIGHT_GAIN_COUNTS_PER_CMD
#ifdef MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD
#define DC_RIGHT_GAIN_COUNTS_PER_CMD MOTOR_CHAR_RIGHT_GAIN_COUNTS_PER_CMD
#else
#define DC_RIGHT_GAIN_COUNTS_PER_CMD MOTOR_CHAR_GAIN_COUNTS_PER_CMD
#endif
#endif

#ifndef DC_LEFT_GAIN_RPM_PER_CMD
#ifdef MOTOR_CHAR_LEFT_GAIN_RPM_PER_CMD
#define DC_LEFT_GAIN_RPM_PER_CMD MOTOR_CHAR_LEFT_GAIN_RPM_PER_CMD
#else
#define DC_LEFT_GAIN_RPM_PER_CMD MOTOR_CHAR_GAIN_RPM_PER_CMD
#endif
#endif

#ifndef DC_RIGHT_GAIN_RPM_PER_CMD
#ifdef MOTOR_CHAR_RIGHT_GAIN_RPM_PER_CMD
#define DC_RIGHT_GAIN_RPM_PER_CMD MOTOR_CHAR_RIGHT_GAIN_RPM_PER_CMD
#else
#define DC_RIGHT_GAIN_RPM_PER_CMD MOTOR_CHAR_GAIN_RPM_PER_CMD
#endif
#endif

#ifndef DC_LEFT_TAU_S
#ifdef MOTOR_CHAR_LEFT_TAU_S
#define DC_LEFT_TAU_S MOTOR_CHAR_LEFT_TAU_S
#else
#define DC_LEFT_TAU_S MOTOR_CHAR_TAU_S
#endif
#endif

#ifndef DC_RIGHT_TAU_S
#ifdef MOTOR_CHAR_RIGHT_TAU_S
#define DC_RIGHT_TAU_S MOTOR_CHAR_RIGHT_TAU_S
#else
#define DC_RIGHT_TAU_S MOTOR_CHAR_TAU_S
#endif
#endif

#ifndef DC_LEFT_LATENCY_S
#ifdef MOTOR_CHAR_LEFT_LATENCY_S
#define DC_LEFT_LATENCY_S MOTOR_CHAR_LEFT_LATENCY_S
#else
#define DC_LEFT_LATENCY_S MOTOR_CHAR_LATENCY_S
#endif
#endif

#ifndef DC_RIGHT_LATENCY_S
#ifdef MOTOR_CHAR_RIGHT_LATENCY_S
#define DC_RIGHT_LATENCY_S MOTOR_CHAR_RIGHT_LATENCY_S
#else
#define DC_RIGHT_LATENCY_S MOTOR_CHAR_LATENCY_S
#endif
#endif
