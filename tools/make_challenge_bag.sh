#!/usr/bin/env bash
# Record a "challenge bag" by running the bringup with teleop or a scenario.
set -euo pipefail
SCENARIO="${1:-scenarios/crossing_targets.yaml}"
WORKDIR="${2:-out/challenge}"
RECORD="${3:-true}"

mkdir -p "$WORKDIR"
echo "[challenge] recording to $WORKDIR"
ros2 launch aura_examples bringup.launch.py scenario:="$SCENARIO" workdir:="$WORKDIR" record:="$RECORD"
echo "[challenge] done. Bag at $WORKDIR/bag_*"
