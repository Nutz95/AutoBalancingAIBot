from __future__ import annotations

from pathlib import Path


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
