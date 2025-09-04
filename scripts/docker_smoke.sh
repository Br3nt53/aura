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
  echo '[smoke] Running the single evaluation...'
  mkdir -p out/tmp
  if [ -f tools/run_single.py ]; then
    if [ -f scenarios/crossing_targets.yaml ]; then
      python3 tools/run_single.py --scenario scenarios/crossing_targets.yaml --out out/tmp/metrics.json
    else
      python3 tools/run_single.py --out out/tmp/metrics.json
    fi
    test -s out/tmp/metrics.json
  else
    echo 'tools/run_single.py not found'; exit 3
  fi"

echo "[ok] smoke passed; metrics present at out/tmp/metrics.json"