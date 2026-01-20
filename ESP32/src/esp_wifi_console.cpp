#include "esp_wifi_console.h"
#include "../config/board_config.h"
#include "logging.h"
#include <Preferences.h>
#include <WiFi.h>
#include <cstring>
// FreeRTOS critical sections for the queue
#include "serial_commands.h"
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifndef WIFI_CONSOLE_ENABLED
// If not enabled, provide no-op implementations
namespace abbot {
namespace wifi_console {
void begin() {}
void loop() {}
void sendLine(const char *) {}
void connectNow() {}
void disconnectNow() {}
void getDiagnostics(char *buf, size_t bufLen) {
  if (!buf || bufLen == 0)
    return;
  buf[0] = '\0';
}
} // namespace wifi_console
} // namespace abbot
#else

namespace abbot {
namespace wifi_console {

static Preferences s_prefs;
static WiFiServer *s_server = nullptr;
static WiFiClient s_client;
static const uint16_t kPort = 2333;
static String s_ssid;
static String s_pass;
static bool s_started = false;
// async queue state and rate-limiter
static uint32_t s_lastSendMs = 0;
static const int kQueueSize = 128; // Réduit pour libérer de la RAM pour le BLE
static const int kLineMax = 512;
static char s_queue[kQueueSize][kLineMax];
static int s_q_head = 0;
static int s_q_tail = 0;
static int s_q_count = 0;
static int s_drops = 0; // number of dropped lines when queue is full
static portMUX_TYPE s_queue_mux = portMUX_INITIALIZER_UNLOCKED;
static const uint32_t kMinMsBetweenSends = 5; // Moins agressif pour laisser du CPU au BLE
static const int kMaxPerDrain = 8; // Moins agressif pour laisser du CPU au BLE
// Reconnect behaviour
static uint32_t s_lastConnectAttemptMs = 0;
static const uint32_t kReconnectIntervalMs =
    10000; // try reconnect every 10s when not connected
// Throttle "periodic reconnect skipped" logs to avoid flooding the serial
static uint32_t s_lastSkippedLogMs = 0;
// Throttle interval for skipped reconnect logs (ms)
static const uint32_t kSkippedLogThrottleMs = 60000;
// Protects access to reconnect/timer state used from loop() and diagnostics
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

static void loadCredentials() {
  if (s_prefs.begin("abbot", true)) {
    s_ssid = s_prefs.getString("wifi_ssid", "");
    s_pass = s_prefs.getString("wifi_pass", "");
    s_prefs.end();
  }
}

static void startServer() {
  if (s_server)
    return;
  s_server = new WiFiServer(kPort);
  s_server->begin();
  LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
             "WIFI-CONSOLE: server listening on port %u\n", (unsigned)kPort);
}

void begin() {
  if (s_started)
    return;
  s_started = true;
  loadCredentials();
  if (s_ssid.length() > 0) {
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "WIFI-CONSOLE: attempting connect to SSID '%s'\n",
               s_ssid.c_str());
    // Ensure WiFi subsystem is in station mode before starting network
    // operations
    WiFi.mode(WIFI_STA);
    // WiFi.setSleep(false); // Disable WiFi power save to prevent timing jitter
    WiFi.begin(s_ssid.c_str(), s_pass.c_str());
    // Non-blocking: we'll poll in loop() for connection; start server once
    // connected
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "WIFI-CONSOLE: no stored SSID (use WIFI SET commands)");
  }
  // Do not start the TCP server here when no credentials are present —
  // starting the server before the TCP/IP stack is initialized can cause
  // lwIP assertions (Invalid mbox). The server will be started when the
  // WiFi status becomes WL_CONNECTED in loop().
}

void connectNow() {
  loadCredentials();
  if (s_ssid.length() > 0) {
    // Ensure WiFi is in station mode before connecting
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "WIFI-CONSOLE: connectNow -> SSID='%s' (len=%u)\n",
               s_ssid.c_str(), (unsigned)s_ssid.length());
    WiFi.begin(s_ssid.c_str(), s_pass.c_str());
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "WIFI-CONSOLE: connectNow called but no stored SSID");
  }
}

void disconnectNow() {
  if (s_client)
    s_client.stop();
  WiFi.disconnect(true, true);
}

void loop() {
  // Poll WiFi connection state and accept clients
  uint32_t now = millis();
  static int s_prevStatus = WL_DISCONNECTED;
  int curStatus = WiFi.status();
  if (curStatus != s_prevStatus) {
    // status changed
    if (curStatus == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                 "WIFI-CONSOLE: connected, IP=%s\n", ip.toString().c_str());
      // ensure server is running
      startServer();
    } else {
      LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                 "WIFI-CONSOLE: WiFi status changed: %d\n", curStatus);
    }
    s_prevStatus = curStatus;
  }

  // If not connected, attempt periodic reconnects (non-aggressive)
  if (curStatus != WL_CONNECTED) {
    if (s_ssid.length() > 0 &&
        (now - s_lastConnectAttemptMs) >= kReconnectIntervalMs) {
      // Only attempt a new begin() when we're in a state that allows a fresh
      // connection attempt (avoid calling begin() repeatedly while in
      // CONNECTING/SCANNING/IDLE which can interfere with the network stack).
      if (curStatus == WL_DISCONNECTED || curStatus == WL_CONNECT_FAILED) {
        s_lastConnectAttemptMs = now;
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "WIFI-CONSOLE: periodic reconnect attempt to SSID='%s'\n",
                   s_ssid.c_str());
        // Only set station mode if STA bit isn't already enabled to avoid
        // toggling AP state unexpectedly.
        if ((WiFi.getMode() & WIFI_MODE_STA) == 0) {
          WiFi.mode(WIFI_MODE_STA);
        }
        WiFi.begin(s_ssid.c_str(), s_pass.c_str());
      } else {
        // Avoid emitting the skipped message repeatedly — only log at most
        // once per `kSkippedLogThrottleMs` and advance the last-attempt
        // timestamp so we don't re-evaluate immediately on the next loop
        // iteration. Protect updates so diagnostics reading won't see torn
        // values.
        portENTER_CRITICAL(&s_state_mux);
        if ((now - s_lastSkippedLogMs) >= kSkippedLogThrottleMs) {
          LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                     "WIFI-CONSOLE: periodic reconnect skipped (status=%d)\n",
                     curStatus);
          s_lastSkippedLogMs = now;
        }
        // Move the last-attempt time forward to defer the next check and
        // avoid tight repeated evaluations while the stack is in a
        // transient state.
        s_lastConnectAttemptMs = now;
        portEXIT_CRITICAL(&s_state_mux);
      }
    }
  }

  if (s_server) {
    if (!s_client || !s_client.connected()) {
      WiFiClient c = s_server->available();
      if (c) {
        if (s_client)
          s_client.stop();
        s_client = c;
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "WIFI-CONSOLE: client connected");
      }
    } else {
      // read incoming data
      if (s_client.available()) {
        String line = s_client.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
          // Log/echo the received line to maintain ordering: emit a small
          // informative log and echo back to the TCP client *before*
          // invoking the command processor so the client sees the RX
          // acknowledgement prior to the command's output.
          LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "WIFI-CONSOLE: rx '%s'\n",
                     line.c_str());
          // Echo immediately to the connected client so the sender sees
          // an ACK without waiting for the queued logger drain.
          if (s_client && s_client.connected()) {
            s_client.print("> ");
            s_client.print(line);
            s_client.print("\r\n");
          }
          // Now inject into the serial command processor (will generate logs)
          abbot::serialcmds::receiveRemoteLine(line);
        }
      }
    }
  }

  // Drain queued outgoing lines to the client with rate limiting.
  if (s_client && s_client.connected()) {
    uint32_t now = millis();
    int sent = 0;
    while (sent < kMaxPerDrain) {
      bool have = false;
      // quick check and pop under lock
      portENTER_CRITICAL(&s_queue_mux);
      if (s_q_count > 0 && (now - s_lastSendMs) >= kMinMsBetweenSends) {
        // copy out one line
        char tmp[kLineMax];
        strncpy(tmp, s_queue[s_q_head], kLineMax);
        tmp[kLineMax - 1] = '\0';
        s_q_head = (s_q_head + 1) % kQueueSize;
        s_q_count--;
        have = true;
        portEXIT_CRITICAL(&s_queue_mux);
        // send without holding queue lock
        s_client.print(tmp);
        s_client.print("\r\n");
        s_lastSendMs = now;
        sent++;
        now = millis();
      } else {
        portEXIT_CRITICAL(&s_queue_mux);
        break;
      }
    }
  }
}

void sendLine(const char *line) {
  if (!line)
    return;
  // Enqueue into the ring buffer quickly. If full, drop oldest to make room.
  portENTER_CRITICAL(&s_queue_mux);
  if (s_q_count >= kQueueSize) {
    // drop oldest
    s_q_head = (s_q_head + 1) % kQueueSize;
    s_q_count--;
    s_drops++;
  }
  // copy up to kLineMax-1 and ensure null-termination
  strncpy(s_queue[s_q_tail], line, kLineMax - 1);
  s_queue[s_q_tail][kLineMax - 1] = '\0';
  s_q_tail = (s_q_tail + 1) % kQueueSize;
  s_q_count++;
  portEXIT_CRITICAL(&s_queue_mux);
}

IPAddress getClientIP() {
    if (s_client && s_client.connected()) {
        return s_client.remoteIP();
    }
    return INADDR_NONE;
}

void getDiagnostics(char *buf, size_t bufLen) {
  if (!buf || bufLen == 0)
    return;
  int qcount = 0;
  int drops = 0;
  int head = 0;
  int tail = 0;
  portENTER_CRITICAL(&s_queue_mux);
  qcount = s_q_count;
  drops = s_drops;
  head = s_q_head;
  tail = s_q_tail;
  portEXIT_CRITICAL(&s_queue_mux);
  int status = WiFi.status();
  IPAddress ip = WiFi.localIP();
  char ipbuf[32] = {0};
  ip.toString().toCharArray(ipbuf, sizeof(ipbuf));
  const char *statusStr = "UNKNOWN";
  switch (status) {
  case WL_NO_SHIELD:
    statusStr = "NO_SHIELD";
    break;
  case WL_IDLE_STATUS:
    statusStr = "IDLE";
    break;
  case WL_NO_SSID_AVAIL:
    statusStr = "NO_SSID_AVAIL";
    break;
  case WL_SCAN_COMPLETED:
    statusStr = "SCAN_COMPLETED";
    break;
  case WL_CONNECTED:
    statusStr = "CONNECTED";
    break;
  case WL_CONNECT_FAILED:
    statusStr = "CONNECT_FAILED";
    break;
  case WL_CONNECTION_LOST:
    statusStr = "CONNECTION_LOST";
    break;
  case WL_DISCONNECTED:
    statusStr = "DISCONNECTED";
    break;
  default:
    break;
  }
  // Read potentially concurrently-updated timestamps under the state mutex
  // to avoid torn reads when diagnostics are requested from another task.
  uint32_t lastSendMsLoc = 0;
  uint32_t lastConnAttemptMsLoc = 0;
  portENTER_CRITICAL(&s_state_mux);
  lastSendMsLoc = s_lastSendMs;
  lastConnAttemptMsLoc = s_lastConnectAttemptMs;
  portEXIT_CRITICAL(&s_state_mux);

  snprintf(buf, bufLen,
           "status=%s(%d) ip=%s queue=%d head=%d tail=%d drops=%d "
           "lastSendMs=%lu lastConnAttemptMs=%lu",
           statusStr, status, ipbuf, qcount, head, tail, drops,
           (unsigned long)lastSendMsLoc, (unsigned long)lastConnAttemptMsLoc);
}

} // namespace wifi_console
} // namespace abbot

#endif
