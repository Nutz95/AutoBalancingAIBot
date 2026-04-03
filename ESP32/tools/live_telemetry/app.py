from __future__ import annotations

import argparse
from pathlib import Path

from .console import CommandProfileService, ConfigSnapshotStore, ConsoleSession, LogBuffer
from .network import TelemetryReceiver
from .state import DashboardState
from .telemetry import TelemetryPacketParser, TelemetrySignalProcessor
from .web import AssetRepository, DashboardHttpServer


class LiveTelemetryApplication:
    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        self._parser = TelemetryPacketParser()
        self._processor = TelemetrySignalProcessor()
        self._state = DashboardState(window_sec=args.window_sec, processor=self._processor)
        self._log_buffer = LogBuffer()
        self._console_session = ConsoleSession(args.robot_host, args.robot_port, self._log_buffer)
        self._config_store = ConfigSnapshotStore(self._console_session)
        self._command_profiles = CommandProfileService(self._console_session, self._config_store)
        bind_host = "" if args.http_host == "0.0.0.0" else args.http_host
        self._receiver = TelemetryReceiver(self._state, self._parser, bind_host=bind_host, udp_port=args.udp_port)
        assets = AssetRepository(Path(__file__).with_name("assets"))
        self._server = DashboardHttpServer(
            (args.http_host, args.http_port),
            self._state,
            self._console_session,
            self._command_profiles,
            self._config_store,
            self._log_buffer,
            assets,
        )

    def run(self) -> int:
        self._console_session.start()
        self._arm_udp_telemetry_if_needed()
        self._receiver.start()
        print(f"Dashboard listening on http://127.0.0.1:{self._args.http_port}")
        print(f"Robot console target: {self._args.robot_host}:{self._args.robot_port}")
        print(f"UDP telemetry port: {self._args.udp_port} | window: {self._args.window_sec:.1f}s")
        try:
            self._server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            print("\nStopping dashboard…")
        finally:
            self._receiver.stop()
            self._console_session.stop()
            self._server.server_close()
        return 0

    def _arm_udp_telemetry_if_needed(self) -> None:
        if self._args.no_auto_start:
            return
        try:
            if not self._console_session.wait_until_connected(timeout_s=2.0):
                raise OSError("console connection timed out")
            self._command_profiles.ensure_udp_auto()
        except (OSError, TimeoutError) as exc:
            print(f"Warning: could not arm UDP telemetry automatically: {exc}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live web dashboard for ESP32 binary telemetry")
    parser.add_argument("--robot-host", required=True, help="ESP32 Wi-Fi console IP")
    parser.add_argument("--robot-port", type=int, default=2333, help="ESP32 Wi-Fi console TCP port")
    parser.add_argument("--udp-port", type=int, default=8888, help="Local UDP port for binary telemetry")
    parser.add_argument("--http-host", default="0.0.0.0", help="HTTP bind host")
    parser.add_argument("--http-port", type=int, default=8000, help="HTTP bind port")
    parser.add_argument("--window-sec", type=float, default=10.0, help="Sliding visualization window in seconds")
    parser.add_argument("--no-auto-start", action="store_true", help="Do not send SYS TELEM UDP AUTO to the robot")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    return LiveTelemetryApplication(args).run()
