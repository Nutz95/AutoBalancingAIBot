#include "logging.h"
#include <cstring>
#include <cctype>
#include <freertos/semphr.h>
#include <Arduino.h>
#include <cstdarg>

namespace abbot {
namespace log {

static uint32_t g_enabled = 0;
static SemaphoreHandle_t g_log_mutex = nullptr;

void init() {
    // default: enable DEFAULT channel
    g_enabled |= CHANNEL_DEFAULT;
    if (!g_log_mutex) {
        g_log_mutex = xSemaphoreCreateMutex();
        // If creation failed, set to null (already null) — prints will still work
    }
}

void enableChannel(Channel c) {
    g_enabled |= static_cast<uint32_t>(c);
}

void disableChannel(Channel c) {
    g_enabled &= ~static_cast<uint32_t>(c);
}

bool isChannelEnabled(Channel c) {
    return (g_enabled & static_cast<uint32_t>(c)) != 0;
}

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
    if (isChannelEnabled(CHANNEL_DEFAULT)) {
        add("DEFAULT");
    }
    if (pos < bufLen) {
        buf[pos] = '\0';
    }
    return pos;
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
        if (g_log_mutex) {
            xSemaphoreGive(g_log_mutex);
        }
        return;
    }
    if (g_log_mutex) {
        xSemaphoreTake(g_log_mutex, portMAX_DELAY);
    }
    Serial.println(s);
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
    if (g_log_mutex) {
        xSemaphoreGive(g_log_mutex);
    }
}

void lockedPrintf(const char *fmt, ...) {
    if (!fmt) {
        return;
    }
    char tmp[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (g_log_mutex) {
        xSemaphoreTake(g_log_mutex, portMAX_DELAY);
    }
    Serial.print(tmp);
    if (g_log_mutex) {
        xSemaphoreGive(g_log_mutex);
    }
}

} // namespace log
} // namespace abbot
