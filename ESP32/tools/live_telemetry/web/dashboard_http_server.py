from __future__ import annotations

from http.server import ThreadingHTTPServer

from ..capture import CaptureManager
from ..console import CommandProfileService, ConfigSnapshotStore, ConsoleSession, LogBuffer
from ..state import DashboardState
from .asset_repository import AssetRepository
from .dashboard_request_handler import DashboardRequestHandler


class DashboardHttpServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        state: DashboardState,
        console_session: ConsoleSession,
        command_profiles: CommandProfileService,
        config_store: ConfigSnapshotStore,
        log_buffer: LogBuffer,
        capture_manager: CaptureManager,
        assets: AssetRepository,
    ) -> None:
        super().__init__(server_address, DashboardRequestHandler)
        self.state = state
        self.console_session = console_session
        self.command_profiles = command_profiles
        self.config_store = config_store
        self.log_buffer = log_buffer
        self.capture_manager = capture_manager
        self.assets = assets
