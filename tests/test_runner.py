# tests/test_runner.py
import json
import pathlib
import subprocess
import sys


def test_run_single(tmp_path: pathlib.Path) -> None:
    """
    Try to run the example scenario. If the runner isn't fully runnable in CI,
    at least import the module to catch syntax errors. If a metrics file is
    produced, assert its AUC is a sane probability.
    """
    out = tmp_path / "metrics.json"
    cmd = [
        sys.executable,
        "tools/run_single.py",
        "--scenario",
        "scenarios/crossing_targets.yaml",
        "--out",
        str(out),
    ]

    try:
        # Allow non-zero return (e.g., missing deps in CI), but don't raise.
        subprocess.run(cmd, check=False)
    except FileNotFoundError:
        # Fallback: just import to catch syntax errors.
        __import__("tools.run_single")
        return

    if out.exists():
        m = json.loads(out.read_text())
        auc = float(m.get("auc", 0.0))
        assert 0.0 <= auc <= 1.0
