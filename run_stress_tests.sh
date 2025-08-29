#!/bin/bash

# AURA Fusion Tracker Stress Test Runner
# This script runs a suite of challenging scenarios and reports the performance.

# Exit immediately if a command exits with a non-zero status.
set -e

# --- THE ENVIRONMENT FIX ---
# Source the ROS 2 environment to make the 'ros2' and other commands available.
# This is required because 'docker compose exec' starts a bare shell.
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f /aura_ws/ros2_ws/install/setup.bash ]; then
  source /aura_ws/ros2_ws/install/setup.bash
fi
# --- END FIX ---

# Define test cases from scenarios/stress_test_suite.yaml
TEST_CASES=(
  "high_clutter"
  "high_density_crossing"
  "popup_targets"
  "low_detection_prob"
  "high_positional_noise"
)

# Define paths
SCENARIO_FILE="scenarios/stress_test_suite.yaml"
PARAMS_FILE="config/golden_params.yaml"
BASE_OUT_DIR="out/stress_tests"

# --- Main Script ---
echo "================================="
echo "  AURA Stress Test Suite Runner  "
echo "================================="
echo

# Clean up previous results
if [ -d "$BASE_OUT_DIR" ]; then
  echo "Cleaning up previous results from $BASE_OUT_DIR..."
  rm -rf "$BASE_OUT_DIR"
fi
mkdir -p "$BASE_OUT_DIR"

# Run each test case
for test_case in "${TEST_CASES[@]}"; do
  echo "--- Running Test Case: $test_case ---"
  TEST_OUT_DIR="$BASE_OUT_DIR/$test_case"
  
  ./tools/run_single.py \
    --scenario "$SCENARIO_FILE" \
    --params "$PARAMS_FILE" \
    --out-dir "$TEST_OUT_DIR" \
    --test-case "$test_case"
  
  echo "--- Test Case '$test_case' Complete ---"
  echo
done

# --- Print Summary ---
echo "=========================="
echo "  Stress Test Summary     "
echo "=========================="
printf "%-25s | %-10s\n" "Test Case" "MOTA Score"
echo "----------------------------------------"

for test_case in "${TEST_CASES[@]}"; do
  METRICS_FILE="$BASE_OUT_DIR/$test_case/metrics.json"
  if [ -f "$METRICS_FILE" ]; then
    # Use python to parse JSON robustly
    MOTA_SCORE=$(python3 -c "import json; f = open('$METRICS_FILE'); data = json.load(f); print(f\"{data.get('mota', 0.0):.4f}\")")
    printf "%-25s | %-10s\n" "$test_case" "$MOTA_SCORE"
  else
    printf "%-25s | %-10s\n" "$test_case" "ERROR"
  fi
done
echo "=========================="
echo "All tests complete. Results are in the '$BASE_OUT_DIR' directory."