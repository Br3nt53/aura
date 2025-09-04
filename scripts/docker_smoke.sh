#!/usr/bin/env bash
# docker_smoke.sh — minimal functional smoke in Docker
# Builds local image and runs evaluator-only to assert output exists.
# Usage: ./scripts/docker_smoke.sh
set -euo pipefail
IFS=$'\n\t'
cd "$(dirname "$0")/.."

IMAGE="aura:ci"
echo "[stage] docker build → $IMAGE"
docker build -t "$IMAGE" .

echo "[stage] docker run smoke"
docker run --rm -v "$PWD":/work -w /work "$IMAGE" bash -lc "\
  set -e
  echo '[smoke] Removing host venv and creating a fresh one inside the container...'
  rm -rf .venv
  python3 -m venv .venv
  source .venv/bin/activate

  echo '[smoke] Installing dependencies into the container venv...'
  uv pip install -r requirements.txt
  if [ -f requirements-dev.txt ]; then uv pip install -r requirements-dev.txt; fi

  echo '[smoke] Running the single evaluation...'
  mkdir -p out/tmp
  if [ -f tools/run_single.py ]; then
    if [ -f scenarios/crossing_targets.yaml ]; then
      python tools/run_single.py --scenario scenarios/crossing_targets.yaml --out out/tmp/metrics.json
    else
      python tools/run_single.py --out out/tmp/metrics.json
    fi
    test -s out/tmp/metrics.json
  else
    echo 'tools/run_single.py not found'; exit 3
  fi"

echo "[ok] smoke passed; metrics present at out/tmp/metrics.json"