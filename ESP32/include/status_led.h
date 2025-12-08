// Minimal status LED wrapper API
// Provide a tiny, safe API to control a single RGB NeoPixel/WS2812 LED.
// If no LED is configured for the board, implementations are no-ops.

#pragma once

#include <stdint.h>

// Initialize the LED hardware if present. Safe to call multiple times.
void statusLedInit();

// Set the single LED's color (0-255 each). Non-blocking API.
void statusLedSetColor(uint8_t r, uint8_t g, uint8_t b);

// Turn LED off. Non-blocking API.
void statusLedOff();

// Returns true if an LED implementation is present (useful for testing).
bool statusLedPresent();

// Configuration macros (optional): define these in board config if present.
// - STATUS_LED_PIN: GPIO pin connected to the LED data line
// - STATUS_LED_NUMPIXELS: number of pixels (default 1)

// Update the status LED based on the IMU consumer state. Implemented as a
// convenience helper so callers (e.g. SystemTasks) can delegate LED logic to
// the status LED module. No-op on boards without an LED.
namespace abbot { namespace imu_consumer { struct ConsumerState; } }
void statusLedUpdateFromConsumer(const abbot::imu_consumer::ConsumerState &state);

// Blink the LED red in a loop to indicate a fatal error. This function
// does not return (intended to be used when startup must halt).
void statusLedBlinkErrorLoop();
