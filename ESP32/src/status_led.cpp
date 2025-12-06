// Implementation of the minimal status LED wrapper.
// Uses Adafruit_NeoPixel when `STATUS_LED_PIN` is defined and library
// is available. Otherwise functions are no-ops.

#include "status_led.h"
#include "../config/board_config.h"
#include "imu_consumer_helpers.h"

#if defined(STATUS_LED_PIN)
#include <Adafruit_NeoPixel.h>

#ifndef STATUS_LED_NUMPIXELS
#define STATUS_LED_NUMPIXELS 1
#endif

static Adafruit_NeoPixel *led = nullptr;

void statusLedInit() {
  if (led) return;
  led = new Adafruit_NeoPixel(STATUS_LED_NUMPIXELS, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
  led->begin();
  led->setBrightness(64);
  led->setPixelColor(0, led->Color(0, 0, 0));
  led->show();
}

void statusLedSetColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!led) return;
  led->setPixelColor(0, led->Color(r, g, b));
  led->show();
}

void statusLedOff() {
  if (!led) return;
  led->setPixelColor(0, led->Color(0, 0, 0));
  led->show();
}

bool statusLedPresent() { return led != nullptr; }

void statusLedUpdateFromConsumer(const abbot::imu_consumer::ConsumerState &state) {
  static uint8_t last_r = 0, last_g = 0, last_b = 0;
  uint8_t r = 0, g = 0, b = 0;
  if (state.warmup_samples_remaining > 0) {
    r = 255; g = 0; b = 0; // red while warming
  } else if (state.gyro_bias_initialized) {
    r = 0; g = 255; b = 0; // green when ready
  } else {
    r = 0; g = 0; b = 0; // off otherwise
  }
  if (r != last_r || g != last_g || b != last_b) {
    statusLedSetColor(r, g, b);
    last_r = r; last_g = g; last_b = b;
  }
}

#else // STATUS_LED_PIN not defined: no-op implementations

void statusLedInit() { }
void statusLedSetColor(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
void statusLedOff() { }
bool statusLedPresent() { return false; }

void statusLedUpdateFromConsumer(const abbot::imu_consumer::ConsumerState &state) {
  static uint8_t last_r = 0, last_g = 0, last_b = 0;
  uint8_t r = 0, g = 0, b = 0;
  if (state.warmup_samples_remaining > 0) {
    r = 255; g = 0; b = 0;
  } else if (state.gyro_bias_initialized) {
    r = 0; g = 255; b = 0;
  } else {
    r = 0; g = 0; b = 0;
  }
  if (r != last_r || g != last_g || b != last_b) {
    statusLedSetColor(r, g, b);
    last_r = r; last_g = g; last_b = b;
  }
}

#endif
