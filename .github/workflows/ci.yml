name: CI

on:
  push:
    branches: [ main ]
  pull_request:

permissions:
  contents: read

jobs:
  lint_test:
    runs-on: ubuntu-latest
    timeout-minutes: 15
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-python@v5
        with:
          python-version: "3.11"
          cache: "pip"

      - name: Install dependencies
        run: |
          python -m pip install -U pip
          if [ -f requirements.txt ]; then python -m pip install -r requirements.txt; fi
          # Core CI tools + libs your tests/smoke commonly touch
          python -m pip install pytest ruff black mypy pandas motmetrics pyyaml

      - name: Run unified checks
        env:
          # make smoke non-fatal by default; flip to "1" to enforce
          AURA_CI_NO_SMOKE: "0"
          AURA_CI_STRICT_SMOKE: "0"
        run: |
          python .github/scripts/ci_checks.py

  docker_smoke:
    runs-on: ubuntu-latest
    timeout-minutes: 20
    needs: [lint_test]
    steps:
      - uses: actions/checkout@v4

      - name: Build image
        run: docker build -t aura-ci .

      - name: Smoke run (ROS inside container)
        run: |
          docker run --rm aura-ci /bin/bash -lc '
            set -e
            . /opt/ros/humble/setup.bash || true
            # If a local ROS2 workspace exists in the image, source it.
            if [ -f "ros2_ws/install/local_setup.bash" ]; then . ros2_ws/install/local_setup.bash; fi

            python3 tools/run_single.py \
              --scenario scenarios/crossing_targets.yaml \
              --out out/tmp/metrics.json

            python3 - <<PY
import json, sys, os
p = "out/tmp/metrics.json"
if not os.path.exists(p):
    print("metrics.json not produced", file=sys.stderr)
    sys.exit(1)
m = json.load(open(p))
auc = float(m.get("auc", 0.0))
assert 0.0 <= auc <= 1.0, f"AUC out of range: {auc}"
print("AUC OK:", auc)
PY
          '
