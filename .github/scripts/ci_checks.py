#!/usr/bin/env python3
import os
import sys
import subprocess
import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]


def run(cmd, cwd: pathlib.Path = ROOT) -> None:
    print("+", *cmd)
    env = os.environ.copy()
    # Ensure in-repo packages (evaluation/, tools/, etc.) import cleanly in CI
    env["PYTHONPATH"] = str(ROOT)
    p = subprocess.run(cmd, cwd=str(cwd), env=env)
    p.check_returncode()


def lint() -> None:
    run(["ruff", "check", "."])
    run(["black", "--check", "."])
    # honor repo mypy.ini (you already have it in root)
    run(["mypy", "--config-file", "mypy.ini", "."])


def tests() -> None:
    run(["pytest", "-q"])


def smoke(strict: bool = False) -> None:
    # Optional mini smoke so CI can validate tooling pathing and JSON output
    out = ROOT / "out" / "tmp" / "metrics.json"
    out.parent.mkdir(parents=True, exist_ok=True)

    run(
        [
            sys.executable,
            "tools/run_single.py",
            "--scenario",
            "scenarios/crossing_targets.yaml",
            "--out",
            str(out),
        ]
    )

    if out.exists():
        data = json.loads(out.read_text())
        print("[SMOKE] Keys:", list(data.keys()))
        mota = float(data.get("mota", 0.0))
        assert 0.0 <= mota <= 1.0, f"MOTA out of range: {mota}"
    elif strict:
        raise SystemExit("Smoke failed: metrics.json not produced")


def main() -> None:
    print()
    lint()
    print()
    tests()
    print()
    # Make smoke non-fatal by default; toggle via env
    do_smoke = os.environ.get("AURA_CI_NO_SMOKE", "0") != "1"
    strict = os.environ.get("AURA_CI_STRICT_SMOKE", "0") == "1"
    if do_smoke:
        smoke(strict=strict)


if __name__ == "__main__":
    main()
