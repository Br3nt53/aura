#!/usr/bin/env bash
# test_all.sh — one-click local test harness for Aura
# Runs: lint/type/test → sweep → evaluator-only → tiny BO → bundles artifacts
# Usage: ./scripts/test_all.sh
set -euo pipefail
IFS=$'\n\t'

# 0) repo root
cd "$(dirname "$0")/.."
echo "[info] repo root: $PWD"

# 1) Install dependencies with uv (assumes uv is installed)
echo "[info] ensuring project dependencies are installed with uv..."
uv pip install -r requirements.txt
if [ -f requirements-dev.txt ]; then
  uv pip install -r requirements-dev.txt
fi

# 2) quality gates (do not hard-fail; continue to produce artifacts)
echo "[stage] lint → format-check → type-check → unit tests"
(ruff . || true)
(black --check . || true)
(mypy || true)
(pytest -q || true)

# 3) experiment sweep (generates HTML report)
if [ -f tools/run_experiments.py ] && [ -f tools/experiments.sample.json ]; then
  echo "[stage] sweep run → tools/run_experiments.py"
  (python tools/run_experiments.py --config tools/experiments.sample.json || true)
else
  echo "[skip] sweep run (tools/ or sample config not found)"
fi

# 4) evaluator-only single run (no dataset; auto-GT from scenario YAML if supported)
mkdir -p out/tmp
if [ -f tools/run_single.py ]; then
  if [ -f scenarios/crossing_targets.yaml ]; then
    echo "[stage] evaluator-only single run with scenario"
    (python tools/run_single.py --scenario scenarios/crossing_targets.yaml --out out/tmp/metrics.json || true)
  else
    echo "[stage] evaluator-only single run (no scenario)"
    (python tools/run_single.py --out out/tmp/metrics.json || true)
  fi
else
  echo "[skip] evaluator-only (tools/run_single.py not found)"
fi

# 5) quick Bayesian optimization demo (kept small for speed)
if [ -f tools/optimize_experiments.py ]; then
  if [ -f scenarios/crossing_targets.yaml ]; then
    echo "[stage] BO demo"
    (python tools/optimize_experiments.py --scenario scenarios/crossing_targets.yaml --n_calls 5 || true)
  else
    echo "[stage] BO demo (no scenario)"
    (python tools/optimize_experiments.py --n_calls 5 || true)
  fi
else
  echo "[skip] BO demo (tools/optimize_experiments.py not found)"
fi

# 6) bundle artifacts for support
ART=aura_debug_bundle.tgz
if [ -d out ]; then
  tar -czf "$ART" out || true
  echo "[ok] artifacts bundled → $PWD/$ART"
else
  echo "[warn] out/ not found; no artifacts to bundle"
fi

echo "[done] test_all.sh complete"