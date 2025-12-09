---
change-id: add-wifi-serial-console
---

# Design notes: Wi‑Fi Serial / TCP Console

This document captures rationale and minimal design details for the existing Wi‑Fi TCP console and the recommended host-client behavior.

Firmware behavior (already implemented):
- Credentials stored in NVS keys `wifi_ssid` and `wifi_pass` are read at boot.
- If present, the firmware attempts to connect using `WiFi.begin()` and polls `WiFi.status()`.
- When connected, the firmware starts a simple TCP server on port 2333.
- Logging is forwarded to an in-memory ring buffer and sent to the client by a rate-limited drain loop; oldest lines are dropped if queue full.

Host-client behavior (recommended):
- Use `SO_KEEPALIVE` to allow OS to detect dead peers.
- Reconnect on send/receive failure using exponential backoff.
- Preserve the last-line send on failure and resend once the socket is re-established.

Security considerations:
- NVS storage is not encrypted; document that credentials are stored in plaintext.
- TCP console has no authentication; it is intended for trusted dev networks. Adding auth is listed as an optional future task.
