#include "logging.h"
#include "esp_wifi_console.h"
#include <Arduino.h>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <freertos/semphr.h>

namespace abbot {
namespace log {

static uint32_t g_enabled = 0;
static SemaphoreHandle_t g_log_mutex = nullptr;
static uint32_t g_mask_stack[8];
static int g_mask_stack_top = 0;

void init() {
  // default: enable DEFAULT channel
  g_enabled |= CHANNEL_DEFAULT;
  if (!g_log_mutex) {
    g_log_mutex = xSemaphoreCreateMutex();
    // If creation failed, set to null (already null) — prints will still work
  }
}

void enableChannel(Channel c) { g_enabled |= static_cast<uint32_t>(c); }

void disableChannel(Channel c) { g_enabled &= ~static_cast<uint32_t>(c); }

bool isChannelEnabled(Channel c) {
  return (g_enabled & static_cast<uint32_t>(c)) != 0;
}

bool toggleChannel(Channel c) {
  uint32_t mask = static_cast<uint32_t>(c);
  if ((g_enabled & mask) != 0) {
    g_enabled &= ~mask;
    return false;
  } else {
    g_enabled |= mask;
    return true;
  }
}

bool pushChannelMask(uint32_t enabledMask) {
  if (g_mask_stack_top >=
      (int)(sizeof(g_mask_stack) / sizeof(g_mask_stack[0]))) {
    return false;
  }
  // save current and set new
  g_mask_stack[g_mask_stack_top++] = g_enabled;
  g_enabled = enabledMask;
  return true;
}

bool popChannelMask() {
  if (g_mask_stack_top <= 0) {
    return false;
  }
  g_enabled = g_mask_stack[--g_mask_stack_top];
  return true;
}

uint32_t getEnabledMask() { return g_enabled; }

void setEnabledMask(uint32_t m) { g_enabled = m; }

static bool iequals(const char *a, const char *b) {
  while (*a && *b) {
    if (std::toupper((unsigned char)*a) != std::toupper((unsigned char)*b)) {
      return false;
    }
    ++a;
    ++b;
  }
  if (*a == *b) {
    return true;
  }
  return false;
}

Channel channelFromString(const char *name) {
  if (!name) {
    return static_cast<Channel>(0);
  }
  if (iequals(name, "TUNING")) {
    return CHANNEL_TUNING;
  }
  if (iequals(name, "BLE")) {
    return CHANNEL_BLE;
  }
  if (iequals(name, "IMU")) {
    return CHANNEL_IMU;
  }
  if (iequals(name, "MOTOR")) {
    return CHANNEL_MOTOR;
  }
  if (iequals(name, "BALANCER")) {
    return CHANNEL_BALANCER;
  }
  if (iequals(name, "DEFAULT")) {
    return CHANNEL_DEFAULT;
  }
  return static_cast<Channel>(0);
}

size_t listEnabledChannels(char *buf, size_t bufLen) {
  size_t pos = 0;
  auto add = [&](const char *s) {
    if (pos != 0 && pos + 1 < bufLen) {
      buf[pos++] = ',';
    }
    while (*s && pos + 1 < bufLen) {
      buf[pos++] = *s++;
    }
  };
  if (isChannelEnabled(CHANNEL_TUNING)) {
    add("TUNING");
  }
  if (isChannelEnabled(CHANNEL_BLE)) {
    add("BLE");
  }
  if (isChannelEnabled(CHANNEL_IMU)) {
    add("IMU");
  }
  if (isChannelEnabled(CHANNEL_MOTOR)) {
    add("MOTOR");
  }
  if (isChannelEnabled(CHANNEL_BALANCER)) {
    add("BALANCER");
  }
  if (isChannelEnabled(CHANNEL_DEFAULT)) {
    add("DEFAULT");
  }
  if (pos < bufLen) {
    buf[pos] = '\0';
  }
  return pos;
}

const char *channelName(Channel c) {
  switch (c) {
  case CHANNEL_TUNING:
    return "TUNING";
  case CHANNEL_BLE:
    return "BLE";
  case CHANNEL_IMU:
    return "IMU";
  case CHANNEL_MOTOR:
    return "MOTOR";
  case CHANNEL_BALANCER:
    return "BALANCER";
  case CHANNEL_DEFAULT:
    return "DEFAULT";
  default:
    return "UNKNOWN";
  }
}

// Locked/serialized printing helpers
void lockedPrint(const char *s) {
  if (!s) {
    return;
  }
  if (g_log_mutex) {
    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  }
  Serial.print(s);
  if (g_log_mutex) {
    xSemaphoreGive(g_log_mutex);
  }
}

void lockedPrintln(const char *s) {
  if (!s) {
    if (g_log_mutex) {
      xSemaphoreTake(g_log_mutex, portMAX_DELAY);
    }
    Serial.println();
    // Forward blank line to wifi console (no-op if disabled)
    abbot::wifi_console::sendLine("");
    if (g_log_mutex) {
      xSemaphoreGive(g_log_mutex);
    }
    return;
  }
  if (g_log_mutex) {
    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  }
  Serial.println(s);
  // Forward line to WiFi console (no-op if module disabled)
  abbot::wifi_console::sendLine(s);
  if (g_log_mutex) {
    xSemaphoreGive(g_log_mutex);
  }
}

void lockedPrint(const String &s) {
  if (g_log_mutex) {
    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  }
  Serial.print(s);
  if (g_log_mutex) {
    xSemaphoreGive(g_log_mutex);
  }
}

void lockedPrintln(const String &s) {
  if (g_log_mutex) {
    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  }
  Serial.println(s);
  // Forward line to WiFi console (no-op if module disabled)
  abbot::wifi_console::sendLine(s.c_str());
  if (g_log_mutex) {
    xSemaphoreGive(g_log_mutex);
  }
}

void lockedPrintf(const char *fmt, ...) {
  if (!fmt) {
    return;
  }
  if (g_log_mutex) {
    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
  }

  static char tmp[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  Serial.print(tmp);
  // Forward formatted output to WiFi console. We forward the raw
  // buffer; if it contains newlines the remote will receive them.
  abbot::wifi_console::sendLine(tmp);

  if (g_log_mutex) {
    xSemaphoreGive(g_log_mutex);
  }
}

void lockedPrintfNonBlocking(const char *fmt, ...) {
  if (!fmt) {
    return;
  }
  
  if (g_log_mutex) {
    // Try to take mutex immediately. If busy, drop the log to protect timing.
    if (xSemaphoreTake(g_log_mutex, 0) != pdTRUE) {
      return;
    }
  }

  static char tmp[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  
  Serial.print(tmp);
  abbot::wifi_console::sendLine(tmp);

  if (g_log_mutex) {
    xSemaphoreGive(g_log_mutex);
  }
}

} // namespace log
} // namespace abbot
