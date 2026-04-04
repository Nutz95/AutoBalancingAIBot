from __future__ import annotations

import json
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlparse

if TYPE_CHECKING:
    from .dashboard_http_server import DashboardHttpServer


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server: "DashboardHttpServer"

    GET_ROUTE_HANDLERS = {
        "/": "_handle_root_request",
        "/api/state": "_handle_state_request",
        "/api/config": "_handle_get_config",
        "/api/logs": "_handle_get_logs",
        "/api/capture/status": "_handle_capture_status",
        "/api/capture/run": "_handle_capture_run",
        "/api/capture/download": "_handle_capture_download",
        "/api/command": "_handle_command",
    }

    POST_ROUTE_HANDLERS = {
        "/api/capture/arm": "_handle_capture_arm",
        "/api/capture/stop": "_handle_capture_stop",
        "/api/capture/delete": "_handle_capture_delete",
        "/api/settings/controller": "_handle_controller_settings",
        "/api/settings/filter": "_handle_filter_settings",
        "/api/motors/enabled": "_handle_motor_enabled",
        "/api/motors/drive": "_handle_motor_drive",
        "/api/system/reboot": "_handle_system_reboot",
        "/api/config/refresh": "_handle_refresh_config",
    }

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path.startswith("/assets/"):
            self._serve_asset(parsed.path.replace("/assets/", "", 1))
            return

        if not self._dispatch_route(self.GET_ROUTE_HANDLERS, parsed.path, parsed.query):
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if not self._dispatch_route(self.POST_ROUTE_HANDLERS, parsed.path, parsed.query):
            self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _dispatch_route(self, route_handlers: dict[str, str], path: str, query: str) -> bool:
        handler_name = route_handlers.get(path)
        if not handler_name:
            return False
        getattr(self, handler_name)(query)
        return True

    def _handle_root_request(self, _query: str = "") -> None:
        self._serve_asset("index.html")

    def _handle_state_request(self, _query: str = "") -> None:
        self._json(self.server.state.snapshot())

    def _handle_command(self, query: str) -> None:
        cmd = parse_qs(query).get("cmd", [""])[0].strip()
        if not cmd:
            self._json({"ok": False, "error": "Missing cmd"}, status=400)
            return
        if not self.server.console_session:
            self._json({"ok": False, "error": "Console disabled"}, status=400)
            return
        try:
            self.server.console_session.send_command(cmd)
            self._json({"ok": True})
        except OSError as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _handle_get_config(self, query: str) -> None:
        force = parse_qs(query).get("force", ["0"])[0] in {"1", "true", "TRUE"}
        try:
            self._json(self.server.config_store.get_snapshot(force=force))
        except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _handle_get_logs(self, query: str) -> None:
        params = parse_qs(query)
        after_sequence = int(params.get("after", ["0"])[0])
        limit = int(params.get("limit", ["200"])[0])
        payload = self.server.log_buffer.snapshot(after_sequence=after_sequence, limit=limit)
        payload["console_connected"] = self.server.console_session.is_connected()
        self._json(payload)

    def _handle_capture_status(self, _query: str = "") -> None:
        self._json(self.server.capture_manager.get_status_payload())

    def _handle_capture_run(self, query: str) -> None:
        capture_id = parse_qs(query).get("capture_id", [""])[0].strip()
        if not capture_id:
            self._json({"ok": False, "error": "Missing capture_id"}, status=400)
            return
        try:
            self._json(self.server.capture_manager.get_capture_payload(capture_id))
        except FileNotFoundError:
            self._json({"ok": False, "error": f"Unknown capture: {capture_id}"}, status=404)

    def _handle_capture_download(self, query: str) -> None:
        params = parse_qs(query)
        capture_id = params.get("capture_id", [""])[0].strip()
        kind = params.get("kind", [""])[0].strip()
        if not capture_id or not kind:
            self._json({"ok": False, "error": "Missing capture_id or kind"}, status=400)
            return
        try:
            target_path, mime_type = self.server.capture_manager.get_download_info(capture_id, kind)
        except ValueError:
            self._json({"ok": False, "error": f"Unsupported download kind: {kind}"}, status=400)
            return
        except FileNotFoundError:
            self._json({"ok": False, "error": f"Unknown capture: {capture_id}"}, status=404)
            return
        self._serve_file(target_path, mime_type, download_name=target_path.name)

    def _handle_capture_arm(self, _query: str = "") -> None:
        payload = self._read_json_body()
        label = payload.get("label")
        capture_mode = payload.get("mode", "manual")
        if label is not None and not isinstance(label, str):
            self._json({"ok": False, "error": "label must be a string"}, status=400)
            return
        if not isinstance(capture_mode, str):
            self._json({"ok": False, "error": "mode must be a string"}, status=400)
            return
        try:
            self._json({"ok": True, "capture": self.server.capture_manager.arm_capture(label, capture_mode)})
        except (RuntimeError, ValueError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=409)

    def _handle_capture_stop(self, _query: str = "") -> None:
        payload = self._read_json_body()
        stop_reason = payload.get("reason")
        if stop_reason is not None and not isinstance(stop_reason, str):
            self._json({"ok": False, "error": "reason must be a string"}, status=400)
            return
        try:
            self._json({"ok": True, "capture": self.server.capture_manager.stop_capture(stop_reason or "manual_stop")})
        except RuntimeError as exc:
            self._json({"ok": False, "error": str(exc)}, status=409)

    def _handle_capture_delete(self, _query: str = "") -> None:
        payload = self._read_json_body()
        capture_id = payload.get("capture_id")
        if not isinstance(capture_id, str) or not capture_id.strip():
            self._json({"ok": False, "error": "capture_id must be a non-empty string"}, status=400)
            return
        try:
            self._json({"ok": True, "capture": self.server.capture_manager.delete_capture(capture_id.strip())})
        except FileNotFoundError:
            self._json({"ok": False, "error": f"Unknown capture: {capture_id}"}, status=404)
        except RuntimeError as exc:
            self._json({"ok": False, "error": str(exc)}, status=409)

    def _handle_controller_settings(self, _query: str = "") -> None:
        self._apply_json_request(
            operation=lambda payload: self.server.command_profiles.apply_controller_settings(payload),
            refresh_sections=["controller", "motors", "system"],
        )

    def _handle_filter_settings(self, _query: str = "") -> None:
        self._apply_json_request(
            operation=lambda payload: self.server.command_profiles.apply_filter_settings(payload),
            refresh_sections=["filter", "system"],
        )

    def _handle_motor_enabled(self, _query: str = "") -> None:
        payload = self._read_json_body()
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            self._json({"ok": False, "error": "enabled must be a boolean"}, status=400)
            return
        try:
            self.server.command_profiles.set_motors_enabled(enabled)
            self._json({"ok": True, "config": self.server.config_store.get_snapshot(force=True, sections=["motors", "system"])})
        except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _handle_motor_drive(self, _query: str = "") -> None:
        payload = self._read_json_body()
        try:
            left_command = float(payload["left"])
            right_command = float(payload["right"])
        except (KeyError, TypeError, ValueError):
            self._json({"ok": False, "error": "left and right must be numeric"}, status=400)
            return
        try:
            self.server.command_profiles.drive_motors(left_command, right_command)
            self._json({"ok": True, "config": self.server.config_store.get_snapshot(force=True, sections=["motors"])})
        except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _handle_system_reboot(self, _query: str = "") -> None:
        try:
            self.server.command_profiles.reboot_system()
            self._json({"ok": True})
        except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _handle_refresh_config(self, _query: str = "") -> None:
        try:
            self._json({"ok": True, "config": self.server.config_store.get_snapshot(force=True)})
        except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _apply_json_request(self, operation: Any, refresh_sections: list[str]) -> None:
        payload = self._read_json_body()
        try:
            operation(payload)
            self._json({"ok": True, "config": self.server.config_store.get_snapshot(force=True, sections=refresh_sections)})
        except ValueError as exc:
            self._json({"ok": False, "error": str(exc)}, status=400)
        except (OSError, TimeoutError, RuntimeError) as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _read_json_body(self) -> dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            return {}
        raw_body = self.rfile.read(content_length)
        return json.loads(raw_body.decode("utf-8"))

    def _serve_asset(self, relative_path: str) -> None:
        try:
            raw, mime = self.server.assets.load(relative_path)
        except FileNotFoundError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self._send_raw_response(raw, mime)

    def _serve_file(self, path: Path, mime_type: str, download_name: str | None = None) -> None:
        try:
            raw = path.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        extra_headers = None if download_name is None else {"Content-Disposition": f'attachment; filename="{download_name}"'}
        self._send_raw_response(raw, mime_type, extra_headers)

    def _send_raw_response(self, raw: bytes, mime_type: str, extra_headers: dict[str, str] | None = None) -> None:
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", mime_type)
            self.send_header("Content-Length", str(len(raw)))
            for header_name, header_value in (extra_headers or {}).items():
                self.send_header(header_name, header_value)
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return

    def _json(self, payload: dict[str, Any], status: int = 200) -> None:
        raw = json.dumps(payload).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return
