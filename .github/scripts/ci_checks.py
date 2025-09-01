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
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    p = subprocess.run(cmd, cwd=str(cwd), env=env)
    if check and p.returncode != 0:
        raise SystemExit(p.returncode)
    return p.returncode


def lint() -> None:
    run(["ruff", "check", "."])
    run(["black", "--check", "."])
    run(["mypy", "--config-file", "mypy.ini", "."])


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
