#!/usr/bin/env python3
"""
Send one or more plain-text commands to the ESP32 Wi-Fi console and print the response.

Examples:
  python tools/wifi_send_command.py --host 192.168.1.159 "BALANCE LQR GAINS"
  python tools/wifi_send_command.py --host 192.168.1.159 "FILTER STATUS" "SYS STATUS"
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import time


def recv_until_quiet(sock: socket.socket, quiet_time_s: float, max_wait_s: float) -> list[str]:
    deadline = time.monotonic() + max_wait_s
    last_rx = time.monotonic()
    buffer = b""
    lines: list[str] = []

    while time.monotonic() < deadline:
        timeout = min(0.1, max(0.0, deadline - time.monotonic()))
        rlist, _, _ = select.select([sock], [], [], timeout)
        if rlist:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buffer += chunk
            last_rx = time.monotonic()
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="ignore").rstrip("\r")
                if line:
                    lines.append(line)
        elif lines and (time.monotonic() - last_rx) >= quiet_time_s:
            break

    if buffer.strip():
        lines.append(buffer.decode("utf-8", errors="ignore").strip())
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Send one-shot commands to the ESP32 Wi-Fi console")
    parser.add_argument("commands", nargs="+", help="One or more commands to send")
    parser.add_argument("--host", required=True, help="ESP32 IP address")
    parser.add_argument("--port", type=int, default=2333, help="TCP console port (default: 2333)")
    parser.add_argument("--connect-timeout", type=float, default=5.0, help="Socket connect timeout in seconds")
    parser.add_argument("--quiet-time", type=float, default=0.35, help="Stop reading after this much silence once data has started arriving")
    parser.add_argument("--max-wait", type=float, default=2.0, help="Maximum time to wait for a response per command")
    args = parser.parse_args()

    try:
        with socket.create_connection((args.host, args.port), timeout=args.connect_timeout) as sock:
            sock.setblocking(False)
            for command in args.commands:
                if not command.endswith("\n"):
                    command += "\n"
                sock.sendall(command.encode("utf-8"))
                print(f"> {command.rstrip()}" )
                lines = recv_until_quiet(sock, quiet_time_s=args.quiet_time, max_wait_s=args.max_wait)
                if lines:
                    for line in lines:
                        print(line)
                else:
                    print("(no response)")
    except OSError as exc:
        print(f"Connection failed: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())