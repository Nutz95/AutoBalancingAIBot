#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../" && pwd -P)"
PY="python"

echo "Running planner -> coder -> verifier (dry-run)"

"$PY" "$ROOT/tools/agents/run_agent.py" --config "$ROOT/.github/agents/Plan.agent.md" --out "$ROOT/artifacts/plan_out.json"
"$PY" "$ROOT/tools/agents/run_agent.py" --config "$ROOT/.github/agents/Coder.agent.md" --in "$ROOT/artifacts/plan_out.json" --out "$ROOT/artifacts/coder_out.json"
"$PY" "$ROOT/tools/agents/run_agent.py" --config "$ROOT/.github/agents/Verifier.agent.md" --in "$ROOT/artifacts/coder_out.json" --out "$ROOT/artifacts/verifier_report.json"

echo "Pipeline complete. Artifacts are in $ROOT/artifacts/"
