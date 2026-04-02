from __future__ import annotations

from http.server import ThreadingHTTPServer

from ..network import RobotConsoleClient
from ..state import DashboardState
from .asset_repository import AssetRepository
from .dashboard_request_handler import DashboardRequestHandler


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
