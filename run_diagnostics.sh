#!/bin/bash
# This script is READ-ONLY. It will not change any files.
set -e

echo "--- 🚀 Running AURA Environment Diagnostics ---"

echo ""
echo "--- 1/6: Verifying Key File Integrity (Checksums) ---"
shasum -a 256 evaluation/run_trackeval.py
shasum -a 256 tools/run_single.py
shasum -a 256 docker-compose.yml
shasum -a 256 Dockerfile
echo "--- ✅ File integrity check complete. ---"

echo ""
echo "--- 2/6: Checking Docker and Docker Compose Versions ---"
docker --version
docker-compose --version
echo "--- ✅ Version check complete. ---"


echo ""
echo "--- 3/6: Checking Docker State (Containers & Images) ---"
echo "--- Running 'docker ps -a' ---"
docker ps -a
echo "--- Running 'docker images | grep aura' ---"
docker images | grep 'aura'
echo "--- ✅ Docker state check complete. ---"

echo ""
echo "--- 4/6: Inspecting Environment Inside the Container ---"
# Ensure the container is running
if [ ! "$(docker ps -q -f name=aura)" ]; then
    echo "Attempting to start 'aura' container..."
    docker-compose up -d
fi
docker-compose exec aura bash -c "
set -e
echo '--- INSIDE CONTAINER: Checking Python and packages ---'
which python3
python3 --version
uv pip list | grep 'trackeval\|numpy'

echo '--- INSIDE CONTAINER: Checking working directory and permissions ---'
pwd
ls -la
"
echo "--- ✅ Container inspection complete. ---"

echo ""
echo "--- 5/6: Manually Simulating the Test Setup Inside the Container ---"
docker-compose exec aura bash -c "
set -e
echo '--- INSIDE CONTAINER: Wiping old test directory ---'
rm -rf out/diag_test
mkdir -p out/diag_test

echo '--- INSIDE CONTAINER: Generating GT and Pred files ONLY ---'
# This runs the first part of the script to create the necessary files for the evaluator
SKIP_ROS=1 python3 -c '
from tools.run_single import generate_ground_truth, synthesize_predictions_from_gt, resolve_paths, parse_args;
from pathlib import Path;
# Create a dummy args object
class Args:
    out_dir = "out/diag_test"
    scenario = "scenarios/crossing_targets.yaml"
    out = None
    params = None
    test_case = None
args = Args()
gt_path, pred_path, _ = resolve_paths(args)
generate_ground_truth(Path(args.scenario), gt_path)
synthesize_predictions_from_gt(gt_path, pred_path)
'
echo '--- INSIDE CONTAINER: Listing the generated directory structure ---'
ls -lR out/diag_test/
"
echo "--- ✅ Manual test setup complete. ---"


echo ""
echo "--- 6/6: Running the failing evaluation script with VERBOSE trace ---"
echo "This will show every file access and command executed by the evaluation script."
docker-compose exec aura bash -c "
set -ex
echo '--- RUNNING EVALUATION WITH TRACE ---'
python3 evaluation/run_trackeval.py --gt out/diag_test/gt.jsonl --pred out/diag_test/pred.jsonl --out-dir out/diag_test
"

echo ""
echo "--- 🚀 DIAGNOSTICS COMPLETE ---"
echo "Please copy the entire output from the '🚀 Running AURA Environment Diagnostics' line onwards and send it back."
echo ""

