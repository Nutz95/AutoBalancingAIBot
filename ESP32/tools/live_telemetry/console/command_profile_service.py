from __future__ import annotations

import threading
from typing import Any

from .config_snapshot_store import ConfigSnapshotStore
from .console_session import ConsoleSession


class CommandProfileService:
    FLOAT_EPSILON = 1.0e-6

    def __init__(self, console_session: ConsoleSession, config_store: ConfigSnapshotStore) -> None:
        self._console_session = console_session
        self._config_store = config_store
        self._lock = threading.Lock()

    def ensure_udp_auto(self) -> None:
        with self._lock:
            self._console_session.send_commands(["SYS TELEM UDP STOP", "SYS TELEM UDP AUTO"])

    def apply_controller_settings(self, payload: dict[str, Any]) -> None:
        current_controller = self._config_store.get_snapshot().get("controller") or {}
        commands = []
        commands.extend(self._build_mode_commands(payload, current_controller))
        commands.extend(self._build_pid_commands(payload, current_controller))
        commands.extend(self._build_lqr_commands(payload, current_controller))
        commands.extend(self._build_output_commands(payload, current_controller))
        self._send_commands(commands)

    def apply_filter_settings(self, payload: dict[str, Any]) -> None:
        commands: list[str] = []
        current = self._config_store.get_snapshot().get("filter") or {}

        current_filter_name = current.get("current")
        filter_name = payload.get("current")
        if filter_name and filter_name != current_filter_name:
            commands.append(f"FILTER SELECT {filter_name}")

        current_params = current.get("params") or {}
        params_payload = payload.get("params") or {}
        for parameter_name, parameter_value in params_payload.items():
            current_value = current_params.get(parameter_name)
            if current_value is None or not self._float_equal(float(current_value), float(parameter_value)):
                commands.append(f"FILTER SET {parameter_name} {float(parameter_value):.6f}")

        self._send_commands(commands)

    def set_motors_enabled(self, enabled: bool) -> None:
        current = self._config_store.get_snapshot().get("motors") or {}
        if current.get("enabled") == enabled:
            return
        command = "MOTOR ENABLE" if enabled else "MOTOR DISABLE"
        self._send_commands([command])

    def drive_motors(self, left_command: float, right_command: float) -> None:
        commands = [
            f"MOTOR SET LEFT {left_command:.6f}",
            f"MOTOR SET RIGHT {right_command:.6f}",
        ]
        self._send_commands(commands)

    def reboot_system(self) -> None:
        self._send_commands(["SYS REBOOT"])

    def _send_commands(self, commands: list[str]) -> None:
        if not commands:
            return
        with self._lock:
            self._console_session.send_commands(commands)

    def _build_mode_commands(self, payload: dict[str, Any], current_controller: dict[str, Any]) -> list[str]:
        mode = self._normalize_mode(payload.get("mode"))
        if mode and mode != current_controller.get("mode"):
            return [f"BALANCE STRATEGY {mode}"]
        return []

    def _build_pid_commands(self, payload: dict[str, Any], current_controller: dict[str, Any]) -> list[str]:
        pid_payload = payload.get("pid") or {}
        current_pid = current_controller.get("pid") or {}
        if not self._has_float_changes(current_pid, pid_payload, ["kp", "ki", "kd"]):
            return []

        return [
            "BALANCE PID GAINS {kp:.6f} {ki:.6f} {kd:.6f}".format(
                kp=float(pid_payload["kp"]),
                ki=float(pid_payload["ki"]),
                kd=float(pid_payload["kd"]),
            )
        ]

    def _build_lqr_commands(self, payload: dict[str, Any], current_controller: dict[str, Any]) -> list[str]:
        lqr_payload = payload.get("lqr") or {}
        current_lqr = current_controller.get("lqr") or {}
        commands: list[str] = []
        commands.extend(self._build_lqr_gain_commands(lqr_payload, current_lqr))
        commands.extend(self._build_lqr_trim_commands(lqr_payload, current_lqr))
        commands.extend(self._build_lqr_filter_commands(lqr_payload, current_lqr))
        return commands

    def _build_lqr_gain_commands(self, lqr_payload: dict[str, Any], current_lqr: dict[str, Any]) -> list[str]:
        current_lqr_gains = current_lqr.get("gains") or {}
        lqr_gains_payload = lqr_payload.get("gains") or {}
        if not self._has_float_changes(current_lqr_gains, lqr_gains_payload, ["k_pitch", "k_gyro", "k_dist", "k_speed"]):
            return []

        return [
            "BALANCE LQR GAINS {k_pitch:.6f} {k_gyro:.6f} {k_dist:.6f} {k_speed:.6f}".format(
                k_pitch=float(lqr_gains_payload["k_pitch"]),
                k_gyro=float(lqr_gains_payload["k_gyro"]),
                k_dist=float(lqr_gains_payload["k_dist"]),
                k_speed=float(lqr_gains_payload["k_speed"]),
            )
        ]

    def _build_lqr_trim_commands(self, lqr_payload: dict[str, Any], current_lqr: dict[str, Any]) -> list[str]:
        adaptive_trim_enabled = lqr_payload.get("adaptive_trim_enabled")
        if adaptive_trim_enabled is None or adaptive_trim_enabled == current_lqr.get("adaptive_trim_enabled"):
            return []
        return [f"BALANCE LQR TRIM {'ON' if adaptive_trim_enabled else 'OFF'}"]

    def _build_lqr_filter_commands(self, lqr_payload: dict[str, Any], current_lqr: dict[str, Any]) -> list[str]:
        current_filters = current_lqr.get("filters") or {}
        lqr_filters_payload = lqr_payload.get("filters") or {}
        if not self._has_float_changes(current_filters, lqr_filters_payload, ["pitch_rate_lpf_hz", "cmd_lpf_hz"]):
            return []

        return [
            "BALANCE LQR FILTER SET {pitch_rate_lpf_hz:.6f} {cmd_lpf_hz:.6f}".format(
                pitch_rate_lpf_hz=float(lqr_filters_payload["pitch_rate_lpf_hz"]),
                cmd_lpf_hz=float(lqr_filters_payload["cmd_lpf_hz"]),
            )
        ]

    def _build_output_commands(self, payload: dict[str, Any], current_controller: dict[str, Any]) -> list[str]:
        current_output = current_controller.get("output") or {}
        output_payload = payload.get("output") or {}
        commands: list[str] = []
        commands.extend(self._build_scalar_output_commands(output_payload, current_output))
        commands.extend(self._build_motor_gain_commands(output_payload, current_output))
        return commands

    def _build_scalar_output_commands(self, output_payload: dict[str, Any], current_output: dict[str, Any]) -> list[str]:
        commands: list[str] = []
        if self._has_float_changes(current_output, output_payload, ["deadband"]):
            commands.append(f"BALANCE DEADBAND SET {float(output_payload['deadband']):.6f}")
        if self._has_float_changes(current_output, output_payload, ["min_cmd"]):
            commands.append(f"BALANCE MIN_CMD SET {float(output_payload['min_cmd']):.6f}")
        return commands

    def _build_motor_gain_commands(self, output_payload: dict[str, Any], current_output: dict[str, Any]) -> list[str]:
        current_motor_gains = current_output.get("motor_gains") or {}
        motor_gains_payload = output_payload.get("motor_gains") or {}
        if not self._has_float_changes(current_motor_gains, motor_gains_payload, ["left", "right"]):
            return []

        return [
            "BALANCE MOTOR_GAINS SET {left:.6f} {right:.6f}".format(
                left=float(motor_gains_payload["left"]),
                right=float(motor_gains_payload["right"]),
            )
        ]

    @classmethod
    def _normalize_mode(cls, mode: Any) -> str | None:
        if mode is None:
            return None
        normalized_mode = str(mode).strip().upper()
        if normalized_mode not in {"PID", "LQR"}:
            raise ValueError("mode must be PID or LQR")
        return normalized_mode

    @classmethod
    def _float_equal(cls, left: float, right: float) -> bool:
        return abs(left - right) <= cls.FLOAT_EPSILON

    @classmethod
    def _has_float_changes(cls, current: dict[str, Any], incoming: dict[str, Any], keys: list[str]) -> bool:
        if not incoming:
            return False
        return any(
            key in incoming and (key not in current or not cls._float_equal(float(current[key]), float(incoming[key])))
            for key in keys
        )
