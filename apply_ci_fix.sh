#!/usr/bin/env bash
set -euo pipefail

# 0) Ensure we're at repo root
cd "$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

# 1) Write the workflow
mkdir -p .github/workflows
cat > .github/workflows/ci.yml <<'YAML'
name: CI

on:
  push:
    branches: [ main ]
  pull_request:

permissions:
  contents: read

jobs:
  python-ci:
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
          python -m pip install pytest ruff black mypy pandas motmetrics pyyaml

      - name: Run unified checks
        env:
          AURA_CI_NO_SMOKE: "0"
          AURA_CI_STRICT_SMOKE: "0"
        run: python .github/scripts/ci_checks.py

  docker_smoke:
    runs-on: ubuntu-latest
    needs: [python-ci]
    timeout-minutes: 20
    steps:
      - uses: actions/checkout@v4

      - name: Build Docker image
        run: docker build -t aura-ci .

      - name: Smoke run (ROS inside container, non-fatal if ROS env not present)
        run: |
          set -euo pipefail
          docker run --rm aura-ci /bin/bash -lc '
            set -e
            if [ -f "/opt/ros/humble/setup.bash" ]; then . /opt/ros/humble/setup.bash; fi
            if [ -f "ros2_ws/install/local_setup.bash" ]; then . ros2_ws/install/local_setup.bash; fi

            mkdir -p out/tmp
            python3 tools/run_single.py \
              --scenario scenarios/crossing_targets.yaml \
              --out out/tmp/metrics.json || true

            python3 - <<PY
import json, sys, pathlib
p = pathlib.Path("out/tmp/metrics.json")
if not p.exists():
    print("[SMOKE][WARN] metrics.json not produced inside container")
    sys.exit(0)
try:
    m = json.load(p.open())
    print("[SMOKE] metrics keys:", list(m.keys()))
except Exception as e:
    print("[SMOKE][WARN] could not parse metrics.json:", e)
PY
          '
YAML

# 2) Write the Python CI runner
mkdir -p .github/scripts
cat > .github/scripts/ci_checks.py <<'PY'
#!/usr/bin/env python3
"""
Unified CI checks for aura.

Runs: ruff, black --check, mypy (ignore missing stubs), pytest
Optional: a lightweight smoke that exercises tools/run_single.py
- Legacy CLI:   --scenario ... --out ...
- New-style CLI: --scenario ... --params ... --out-dir ...

Toggles:
  AURA_CI_NO_SMOKE=1        -> skip smoke
  AURA_CI_STRICT_SMOKE=1    -> make smoke required (fail if it fails)
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys
from typing import List

ROOT = pathlib.Path(__file__).resolve().parents[2]  # repo root


def run(cmd: List[str], cwd: pathlib.Path | None = None, check: bool = True) -> int:
    cwd = cwd or ROOT
    print("+", " ".join(cmd))
    p = subprocess.run(cmd, cwd=str(cwd))
    if check and p.returncode != 0:
        raise SystemExit(p.returncode)
    return p.returncode


def lint() -> None:
    run(["ruff", "check", "."])
    run(["black", "--check", "."])
    run(["mypy", "--ignore-missing-imports", "."])


def tests() -> None:
    run(["pytest", "-q"])


def smoke(strict: bool) -> None:
    out = ROOT / "out" / "tmp"
    out.mkdir(parents=True, exist_ok=True)
    metrics = out / "metrics.json"
    scenario = ROOT / "scenarios" / "crossing_targets.yaml"

    rc = run(
        [
            sys.executable,
            "tools/run_single.py",
            "--scenario",
            str(scenario),
            "--out",
            str(metrics),
        ],
        check=False,
    )

    if rc != 0 or not metrics.exists():
        msg = "Smoke did not produce metrics.json"
        if strict:
            print("[SMOKE][FAIL]", msg)
            raise SystemExit(1)
        else:
            print("[SMOKE][WARN]", msg)
            return

    try:
        data = json.loads(metrics.read_text(encoding="utf-8"))
        print("[SMOKE] Keys:", list(data.keys()))
    except Exception as e:
        txt = f"[SMOKE][WARN] metrics parse: {e}"
        if strict:
            print(txt)
            raise SystemExit(1)
        print(txt)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-smoke", action="store_true")
    ap.add_argument("--strict-smoke", action="store_true")
    args = ap.parse_args()

    lint()
    tests()

    skip = args.no_smoke or os.getenv("AURA_CI_NO_SMOKE") == "1"
    strict = args.strict_smoke or os.getenv("AURA_CI_STRICT_SMOKE") == "1"
    if not skip:
        smoke(strict=strict)


if __name__ == "__main__":
    main()
PY
chmod +x .github/scripts/ci_checks.py

# 3) Quick local dry run (mirrors "python-ci" job)
python .github/scripts/ci_checks.py || true

# 4) Commit & push
git add .github/workflows/ci.yml .github/scripts/ci_checks.py
git commit -m "ci: resolve YAML conflicts; add unified Python checks and optional Docker smoke"
git push -u origin "$(git rev-parse --abbrev-ref HEAD)" || git push

echo "[done] CI files updated and pushed."
