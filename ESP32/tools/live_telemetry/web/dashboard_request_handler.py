from __future__ import annotations

import json
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from typing import TYPE_CHECKING, Any
from urllib.parse import parse_qs, urlparse

if TYPE_CHECKING:
    from .dashboard_http_server import DashboardHttpServer


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server: "DashboardHttpServer"

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
