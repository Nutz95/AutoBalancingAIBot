---
change-id: add-wifi-serial-console
title: Wi‑Fi serial/TCP console documentation
authors: ["automated-agent"]
status: draft
created: 2025-12-09
---

# Proposal: Document Wi‑Fi Serial / TCP Console and Host Client Behavior

## Overview

This change documents the existing Wi‑Fi serial/TCP console feature in the ESP32 firmware, specifies the expected behavior for configuration and boot-time auto-connect, and recommends a resilient host-side client behavior (keepalive, automatic reconnect with backoff, and resending failed commands).

It includes:
- a short specification of firmware behavior and acceptance scenarios,
- recommended host-client behavior and test scenarios,
- user-facing documentation that will be added to `ESP32/README.md` describing how to set Wi‑Fi credentials from the USB serial console and how to use the included Python client.

## Why

We already ship a lightweight TCP console that forwards logs and accepts textual commands. Users were unclear how to persist Wi‑Fi credentials, how auto-connect works, and how the host-side client should behave when the device is reflashed (connection closes). This proposal clarifies those points and documents the recommended client behavior implemented in `ESP32/tools/wifi_console_client.py`.

## What Changes

- Read `wifi_ssid` and `wifi_pass` keys from NVS on boot and attempt to connect automatically if values are present.
- When connected, start the TCP console server (default port 2333) and accept a single client.
- Provide serial commands to set the SSID and password: `WIFI SET SSID <ssid>` and `WIFI SET PASS <pwd>` (persist to NVS).
- Provide `WIFI STATUS` and `WIFI DIAG` commands to inspect connection state and diagnostics.

## Host-client recommendations

- Enable `SO_KEEPALIVE` on the TCP socket so the kernel can detect stale connections.
- On send failure (BrokenPipe/OSError), the client should remember the last line and attempt to reconnect with exponential backoff, then resend the pending command after reconnect.

## Acceptance criteria

- `ESP32/README.md` is updated with a step-by-step guide to configure Wi‑Fi using the serial menu, instructions to run the Python client, and a note that the firmware auto-connects at boot.
- OpenSpec files are added under `openspec/changes/add-wifi-serial-console/`.
- The example Python client demonstrates keepalive, reconnect/backoff, and resend behavior.
