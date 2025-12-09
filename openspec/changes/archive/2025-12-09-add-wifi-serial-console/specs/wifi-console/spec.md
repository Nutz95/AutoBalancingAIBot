---
title: Wi‑Fi TCP Console Spec
change-id: add-wifi-serial-console
---

## ADDED Requirements

### Requirement: Boot-time auto-connect

The firmware MUST read NVS keys `wifi_ssid` and `wifi_pass` on boot and,
when both values are present, attempt to connect to the configured Wi‑Fi
network. If the connection is established the firmware MUST start the TCP
console server on the configured port (default `2333`).

#### Scenario: Auto-connect and server start

- Given valid credentials stored in NVS under `wifi_ssid`/`wifi_pass`,
- When the device boots,
- Then the device attempts to connect to Wi‑Fi and once `CONNECTED` the
  TCP console server is started and begins accepting a client.

### Requirement: Serial configuration commands

The firmware MUST expose serial commands to persist Wi‑Fi credentials. The
commands `WIFI SET SSID <ssid>` and `WIFI SET PASS <pwd>` MUST store the
values to NVS and return a confirmation message.

#### Scenario: Persisting credentials via USB serial

- Given the device is connected over USB and a serial terminal is open,
- When the user sends `WIFI SET SSID MyNet` and `WIFI SET PASS Secret` as
  plain-text lines,
- Then the firmware persists the values in NVS and replies with a success
  message for each command.

### Requirement: TCP console behavior

When the TCP console is active the firmware MUST forward device logs to the
connected client as text lines and accept plain-text command lines from the
client, passing them to the same command processor used by the USB serial
console.

#### Scenario: Remote command execution

- Given a client connected to the TCP console,
- When the client sends a command line (for example `HELP`),
- Then the firmware must echo/ACK the received line to the client and execute
  the command using the same parser as the USB serial console and send the
  command output back to the client.


### Requirement: Host client resilience (recommendation)

Host clients MUST enable `SO_KEEPALIVE` on the socket to allow the OS to
detect dead peers. On a send or receive failure the host client MUST
attempt reconnects with exponential backoff and preserve the last command
line so it may be resent after a successful reconnect.

#### Scenario: Resend after reconnect

- Given a host client connected and the device is reflashed causing a
  disconnection while the client is sending a line,
- When the send fails with a BrokenPipe/OSError,
- Then the client remembers the failed line, attempts reconnects with
  exponential backoff, and resends the remembered command once the
  connection is re-established.

### Requirement: Documentation

The repository MUST include documentation describing the USB serial commands
for configuring Wi‑Fi, the boot-time auto-connect behavior, and instructions
for running the example Python client located at `ESP32/tools/wifi_console_client.py`.

#### Scenario: Documentation presence

- Given the repository contains `ESP32/README.md`,
- When reviewing the Wi‑Fi console section,
- Then it must describe `WIFI SET SSID` / `WIFI SET PASS`, the auto-connect
  behavior, and usage instructions for the Python client including reconnect
  behavior notes.
