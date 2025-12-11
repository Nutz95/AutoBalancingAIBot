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
#define DC_MIRROR_MODE_ENABLED 1
#endif

// Specify which encoder(s) are present on the platform. Set 1 if present.
#ifndef DC_ENCODER_PRESENT_LEFT
#define DC_ENCODER_PRESENT_LEFT 0
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
#ifndef DC_LEFT_PWM_A_PIN
#define DC_LEFT_PWM_A_PIN 25
#endif
#ifndef DC_LEFT_PWM_B_PIN
#define DC_LEFT_PWM_B_PIN 26
#endif
#ifndef DC_LEFT_EN_A_PIN
#define DC_LEFT_EN_A_PIN 27
#endif
#ifndef DC_LEFT_EN_B_PIN
#define DC_LEFT_EN_B_PIN 14
#endif

#ifndef DC_RIGHT_PWM_A_PIN
#define DC_RIGHT_PWM_A_PIN 32
#endif
#ifndef DC_RIGHT_PWM_B_PIN
#define DC_RIGHT_PWM_B_PIN 33
#endif
#ifndef DC_RIGHT_EN_A_PIN
#define DC_RIGHT_EN_A_PIN 12
#endif
#ifndef DC_RIGHT_EN_B_PIN
#define DC_RIGHT_EN_B_PIN 13
#endif

// PWM frequency / resolution defaults used by DC driver. Adjust as needed.
#ifndef DC_PWM_FREQUENCY_HZ
#define DC_PWM_FREQUENCY_HZ 20000
#endif
#ifndef DC_PWM_RESOLUTION_BITS
#define DC_PWM_RESOLUTION_BITS 8
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
#define DC_LEFT_MOTOR_INVERT 0
#endif
#ifndef DC_RIGHT_MOTOR_INVERT
#define DC_RIGHT_MOTOR_INVERT 0
#endif
