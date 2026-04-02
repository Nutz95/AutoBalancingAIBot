#!/usr/bin/env python3
"""Backward-compatible entrypoint for the live telemetry dashboard."""

from __future__ import annotations

from live_telemetry.app import main


if __name__ == "__main__":
    raise SystemExit(main())
