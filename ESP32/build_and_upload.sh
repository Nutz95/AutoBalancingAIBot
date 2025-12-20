#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd -P)"

usage() {
  echo "Usage: $(basename "$0") [platformio args...]"
  echo
  echo "Runs: pio run && pio run --target upload (in ESP32 folder). Additional args are forwarded to both commands."
  exit 0
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
fi

cd "$ROOT_DIR" || exit 1

if command -v pio >/dev/null 2>&1; then
  PIO_CMD=(pio)
elif command -v platformio >/dev/null 2>&1; then
  PIO_CMD=(platformio)
elif command -v python3 >/dev/null 2>&1 && python3 -c "import importlib.util,sys
spec=importlib.util.find_spec('platformio')
sys.exit(0 if spec else 1)" >/dev/null 2>&1; then
  PIO_CMD=(python3 -m platformio)
elif command -v python >/dev/null 2>&1 && python -c "import importlib.util,sys
spec=importlib.util.find_spec('platformio')
sys.exit(0 if spec else 1)" >/dev/null 2>&1; then
  PIO_CMD=(python -m platformio)
elif [ -x "$HOME/pipx/venvs/platformio/Scripts/platformio.exe" ]; then
  PIO_CMD=("$HOME/pipx/venvs/platformio/Scripts/platformio.exe")
elif [ -x "/c/Users/$USER/pipx/venvs/platformio/Scripts/platformio.exe" ]; then
  PIO_CMD=("/c/Users/$USER/pipx/venvs/platformio/Scripts/platformio.exe")
elif [ -x "$HOME/.local/bin/platformio" ]; then
  PIO_CMD=("$HOME/.local/bin/platformio")
else
  echo "PlatformIO CLI not found. Install via pipx or use 'python -m platformio'." >&2
  exit 2
fi

"${PIO_CMD[@]}" run "$@"
"${PIO_CMD[@]}" run --target upload "$@"
