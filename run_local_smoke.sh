#!/usr/bin/env bash
set -euo pipefail

# Always operate from the repo root
cd "$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

# Ensure required dirs/files exist
mkdir -p scenarios out/test_run
if [ ! -f scenarios/params.min.yaml ]; then
  cat > scenarios/params.min.yaml <<'YAML'
evaluator:
  objective: "mota"
pipeline:
  skip_ros: true
YAML
fi

# Useful env for local runs
export AURA_LOG_LEVEL=INFO
export USE_AURA_EVALUATOR=1
export SKIP_ROS=1

echo "[1/3] Legacy flow -> out/metrics.json"
python3 tools/run_single.py --scenario scenarios/crossing_targets.yaml --out out/metrics.json || true

echo "[2/3] New-style flow -> out/test_run/"
python3 tools/run_single.py \
  --scenario scenarios/crossing_targets.yaml \
  --params   scenarios/params.min.yaml \
  --out-dir  out/test_run || true

echo "[3/3] Hooks + tests (non-fatal locally)"
pre-commit run --all-files || true
pytest -q || true

echo "[done] legacy: out/metrics.json ; new: out/test_run/metrics.json"
