from __future__ import annotations

import json
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from .network import RobotConsoleClient
from .state import DashboardState


class AssetRepository:
    def __init__(self, asset_dir: Path) -> None:
        self._asset_dir = asset_dir
        self._mime_types = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
        }

    def load(self, relative_path: str) -> tuple[bytes, str]:
        safe_relative = relative_path.lstrip("/")
        path = (self._asset_dir / safe_relative).resolve()
        if not str(path).startswith(str(self._asset_dir.resolve())) or not path.is_file():
            raise FileNotFoundError(relative_path)
        return path.read_bytes(), self._mime_types.get(path.suffix, "application/octet-stream")


class DashboardHttpServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        state: DashboardState,
        console: RobotConsoleClient | None,
        assets: AssetRepository,
    ) -> None:
        super().__init__(server_address, DashboardRequestHandler)
        self.state = state
        self.console = console
        self.assets = assets


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server: DashboardHttpServer

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._serve_asset("index.html")
            return
        if parsed.path.startswith("/assets/"):
            self._serve_asset(parsed.path.replace("/assets/", "", 1))
            return
        if parsed.path == "/api/state":
            self._json(self.server.state.snapshot())
            return
        if parsed.path == "/api/command":
            self._handle_command(parsed.query)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _handle_command(self, query: str) -> None:
        cmd = parse_qs(query).get("cmd", [""])[0].strip()
        if not cmd:
            self._json({"ok": False, "error": "Missing cmd"}, status=400)
            return
        if not self.server.console:
            self._json({"ok": False, "error": "Console disabled"}, status=400)
            return
        try:
            lines = self.server.console.send(cmd)
            self._json({"ok": True, "lines": lines})
        except OSError as exc:
            self._json({"ok": False, "error": str(exc)}, status=502)

    def _serve_asset(self, relative_path: str) -> None:
        try:
            raw, mime = self.server.assets.load(relative_path)
        except FileNotFoundError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _json(self, payload: dict[str, Any], status: int = 200) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)
