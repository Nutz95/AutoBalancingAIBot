// logging.h - simple log channel manager
#pragma once
#include <cstdint>
#include <cstddef>
#include <Arduino.h>

// Lightweight logging helpers: prefer using the channel-gated macros below
// so that messages are only evaluated/printed when the channel is enabled.
// Examples:
//   LOG_PRINTLN(CHANNEL_DEFAULT, "Hello world");
//   LOG_PRINTF(CHANNEL_TUNING, "%u,%f\n", ts_ms, pitch_deg);

// Macro notes:
// - These macros check `abbot::log::isChannelEnabled()` before calling
//   `Serial.print/println/printf`, so argument evaluation is skipped when
//   the channel is disabled.
// - `Serial.printf` is used for formatted output (ESP32 Arduino supports it).
#

namespace abbot {
namespace log {

enum Channel : uint32_t {
    CHANNEL_TUNING = 1u << 0,
    CHANNEL_BLE     = 1u << 1,
    CHANNEL_IMU     = 1u << 2,
    CHANNEL_MOTOR   = 1u << 3,
    CHANNEL_DEFAULT = 1u << 4,
};

// Initialize logging manager (no-op, safe to call multiple times)
void init();

// Enable/disable channels
void enableChannel(Channel c);
void disableChannel(Channel c);
bool isChannelEnabled(Channel c);

// Toggle the channel state and return the new state (true = enabled)
bool toggleChannel(Channel c);

// Push a new enabled-channel mask onto an internal stack and set it.
// Use this to temporarily restrict which channels are active. Returns true
// on success (stack not full).
bool pushChannelMask(uint32_t enabledMask);

// Pop the last pushed mask and restore it. Returns true on success (stack
// was not empty).
bool popChannelMask();

// Directly set/get the enabled mask (for advanced usage)
uint32_t getEnabledMask();
void setEnabledMask(uint32_t m);

// Utility: parse channel name (case-insensitive). Returns 0 on failure.
Channel channelFromString(const char *name);

// Return a comma-separated list of enabled channel names into provided buffer.
// bufLen should be large enough; function returns number of bytes written.
size_t listEnabledChannels(char *buf, size_t bufLen);

// Return the canonical name for a channel (literal string, do not free)
const char* channelName(Channel c);

// Print helpers (macros): avoid evaluating arguments when channel disabled.
// These macros call the locked helpers in `abbot::log` so prints are
// serialized with a mutex and routed centrally.
#define LOG_PRINT(ch, ...) \
    do { if (abbot::log::isChannelEnabled(ch)) { abbot::log::lockedPrint(__VA_ARGS__); } } while (0)

#define LOG_PRINTLN(ch, ...) \
    do { if (abbot::log::isChannelEnabled(ch)) { abbot::log::lockedPrintln(__VA_ARGS__); } } while (0)

#define LOG_PRINTF(ch, fmt, ...) \
    do { if (abbot::log::isChannelEnabled(ch)) { abbot::log::lockedPrintf((fmt), ##__VA_ARGS__); } } while (0)

// Locked/serialized print helpers implemented in logging.cpp
void lockedPrint(const char *s);
void lockedPrintln(const char *s);
void lockedPrint(const String &s);
void lockedPrintln(const String &s);
void lockedPrintf(const char *fmt, ...);

} // namespace log
} // namespace abbot

// (No global aliases here — callers should qualify channel names as
// `abbot::log::CHANNEL_*` to avoid accidental macro/namespace collisions.)
