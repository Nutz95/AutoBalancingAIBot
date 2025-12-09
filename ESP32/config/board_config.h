// Board-specific configuration overrides.
// Define STATUS_LED_PIN to enable the status LED implementation.
// Override these values in board-specific headers if needed.

#pragma once

// Uncomment and set the correct GPIO for your board's single NeoPixel/WS2812 data pin.
// Example (Waveshare): 48
 #define STATUS_LED_PIN 48

// Number of pixels (default 1)
 #define STATUS_LED_NUMPIXELS 1

// Enable the optional Wi‑Fi console module. When enabled, the firmware will
// attempt to read stored Wi‑Fi credentials from NVS and start a TCP console
// server allowing remote log/command access. Set to 1 to enable, 0 to disable.
#ifndef WIFI_CONSOLE_ENABLED
#define WIFI_CONSOLE_ENABLED 1
#endif
