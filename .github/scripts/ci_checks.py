#!/usr/bin/env python3
"""
Unified CI checks for the AURA experiments runner.

This script runs a series of checks, including linting, formatting, type checking,
and smoke tests. It is designed to be run from the root of the repository.
"""

import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).parent.parent.parent


def run(cmd: list[str]) -> None:
    """Run a command and check for errors."""
    print(f"\n--- Running: {' '.join(cmd)} ---")
    p = subprocess.run(cmd, cwd=REPO_ROOT)
    p.check_returncode()


def lint() -> None:
    """Run linters and type checkers."""
    print("\n--- Running linters and type checkers ---")
    run(["ruff", "check", "."])
    run(["black", "--check", "."])
    run(["mypy", "--config-file", "mypy.ini", "."])


def smoke(strict: bool = False) -> None:
    """Non-ROS smoke tests."""
    print("\n--- Running smoke tests ---")
    try:
        run(
            [
                sys.executable,
                "tools/run_single.py",
                "--scenario",
                "scenarios/crossing_targets.yaml",
                "--out",
                f"{REPO_ROOT}/out/tmp/metrics.json",
                # Add the new required arguments
                "--rf_weight",
                "0.5",
                "--wall_bonus",
                "0.5",
                "--track_decay_sec",
                "1.0",
            ]
        )
    except subprocess.CalledProcessError as e:
        if strict:
            raise e
        else:
            print(f"WARNING: Smoke test failed: {e}")


def main() -> None:
    """Run all checks."""
    # In CI, we want to fail on any error.
    # Locally, we can be more permissive.
    strict = os.getenv("CI") == "true"

    lint()
    # Basic pytest before more complex smoke tests
    run(["pytest", "-q"])
    smoke(strict=strict)
    print("\n--- All checks passed! ---")


if __name__ == "__main__":
    main()
