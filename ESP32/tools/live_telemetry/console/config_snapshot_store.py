from __future__ import annotations

import threading
import time
from typing import Any

from .console_session import ConsoleSession


class ConfigSnapshotStore:
    DEFAULT_REFRESH_INTERVAL_S = 1.5
    DEFAULT_REQUEST_TIMEOUT_S = 2.0

    _SECTION_COMMANDS = {
        "controller": ("WEBUI GET CONTROLLER", "WEBUI_CONTROLLER_JSON:"),
        "filter": ("WEBUI GET FILTER", "WEBUI_FILTER_JSON:"),
        "motors": ("WEBUI GET MOTORS", "WEBUI_MOTORS_JSON:"),
        "system": ("WEBUI GET SYSTEM", "WEBUI_SYSTEM_JSON:"),
        "logs": ("WEBUI GET LOGS", "WEBUI_LOGS_JSON:"),
    }

    def __init__(
        self,
        console_session: ConsoleSession,
        refresh_interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
        request_timeout_s: float = DEFAULT_REQUEST_TIMEOUT_S,
    ) -> None:
        self._console_session = console_session
        self._refresh_interval_s = refresh_interval_s
        self._request_timeout_s = request_timeout_s
        self._snapshot_lock = threading.Lock()
        self._refresh_lock = threading.Lock()
        self._sections: dict[str, dict[str, Any]] = {}
        self._last_refresh_times: dict[str, float] = {}

    def get_snapshot(self, force: bool = False, sections: list[str] | None = None) -> dict[str, Any]:
        section_names = sections or list(self._SECTION_COMMANDS.keys())
        if self._console_session.is_connected():
            self.refresh_sections(section_names, force=force)
        with self._snapshot_lock:
            payload = {section: self._sections.get(section) for section in self._SECTION_COMMANDS}
        payload["console_connected"] = self._console_session.is_connected()
        return payload

    def refresh_sections(self, sections: list[str], force: bool = False) -> None:
        sections_to_refresh = [section for section in sections if self._needs_refresh(section, force)]
        if not sections_to_refresh:
            return

        with self._refresh_lock:
            for section in sections_to_refresh:
                command, response_prefix = self._SECTION_COMMANDS[section]
                payload = self._console_session.request_json(command, response_prefix, timeout_s=self._request_timeout_s)
                with self._snapshot_lock:
                    self._sections[section] = payload
                    self._last_refresh_times[section] = time.monotonic()

    def _needs_refresh(self, section: str, force: bool) -> bool:
        if force:
            return True
        with self._snapshot_lock:
            if section not in self._sections:
                return True
            last_refresh_time = self._last_refresh_times.get(section, 0.0)
        return (time.monotonic() - last_refresh_time) >= self._refresh_interval_s
