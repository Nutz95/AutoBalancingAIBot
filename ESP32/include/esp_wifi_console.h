#pragma once

#include <Arduino.h>

namespace abbot {
namespace wifi_console {

// Initialize the wifi console module. Safe to call when WIFI_CONSOLE_ENABLED
// is 0 — implementation will be a no-op.
void begin();

// Polling loop to be called periodically from SystemTasks main loop or a task.
void loop();

// Send a single line (null-terminated) to the connected TCP client if any.
void sendLine(const char *line);

// Connect/disconnect helpers exposed for manual control
void connectNow();
void disconnectNow();

// Diagnostics: fill provided buffer with a short status summary
// (null-terminated) bufLen should be large enough (256 recommended).
void getDiagnostics(char *buf, size_t bufLen);

// Notes:
// - The module will attempt periodic reconnects when credentials are
//   present. To avoid flooding serial output, skipped reconnect logs are
//   throttled by default (implementation detail). The reconnect interval
//   and throttle are implementation constants and may be exposed via runtime
//   Preferences in a future update.

} // namespace wifi_console
} // namespace abbot
