#!/usr/bin/env python3
"""
Single-scenario runner for Aura experiments.

- Always synthesizes a tiny GT and prediction set when the ROS path is disabled.
- Hands evaluation off to evaluation/run_trackeval.py
- Guarantees smoke/metrics.json is produced (or raises with clear logs).
"""

from __future__ import annotations

import argparse
import json
import logging
import subprocess
import sys
from pathlib import Path

logger = logging.getLogger("aura.run_single")
logging.basicConfig(level=logging.INFO)


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _run(cmd: list[str]) -> None:
    logger.info("Running: %s", " ".join(cmd))
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        logger.error("stdout:\n%s", proc.stdout)
        logger.error("stderr:\n%s", proc.stderr)
        raise RuntimeError(f"Command failed ({proc.returncode}): {' '.join(cmd)}")
    if proc.stdout.strip():
        logger.info("stdout:\n%s", proc.stdout)


def _synthesize_jsonl(jsonl_path: Path, kind: str) -> None:
    """
    Write a tiny non-empty stream so evaluators never divide-by-zero.
    Schema tolerated by our wrapper: frame,id,x,y,score (w,h optional).
    """
    data = [
        {"frame": 1, "id": 1, "x": 50.0, "y": 50.0, "w": 40, "h": 80, "score": 0.99},
        {"frame": 2, "id": 1, "x": 55.0, "y": 55.0, "w": 40, "h": 80, "score": 0.99},
    ]
    jsonl_path.write_text(
        "\n".join(json.dumps(d) for d in data) + "\n", encoding="utf-8"
    )
    logger.info("Synthesized %s at %s (%d lines)", kind, jsonl_path, len(data))


def evaluate_predictions(
    pred_path: Path,
    gt_path: Path,
    metrics_path: Path,
    include_meta: bool = True,
) -> None:
    """Evaluate predictions via the TrackEval wrapper; writes metrics.json."""
    _ensure_dir(metrics_path.parent)
    out_dir = metrics_path.parent
    cmd = [
        sys.executable,
        "evaluation/run_trackeval.py",
        "--gt",
        str(gt_path),
        "--pred",
        str(pred_path),
        "--out-dir",
        str(out_dir),
    ]
    logger.info("Evaluating with TrackEval wrapper …")
    _run(cmd)
    if not metrics_path.exists():
        raise FileNotFoundError(
            f"Expected metrics at {metrics_path}, but it was not created."
        )
    logger.info("Metrics ready: %s", metrics_path)


def main() -> None:
    p = argparse.ArgumentParser(
        description="Run a single Aura scenario (smoke-friendly)."
    )
    p.add_argument(
        "--scenario",
        required=True,
        help="Path to scenario YAML (unused in smoke synth).",
    )
    p.add_argument("--out-dir", required=True, help="Output directory (e.g., smoke)")
    args = p.parse_args()

    out_dir = Path(args.out_dir)
    _ensure_dir(out_dir)

    gt_path = out_dir / "gt.jsonl"
    pred_path = out_dir / "pred.jsonl"
    metrics_path = out_dir / "metrics.json"

    # Always create minimal-but-valid inputs if they don't exist.
    if not gt_path.exists():
        _synthesize_jsonl(gt_path, "ground-truth")
    if not pred_path.exists():
        _synthesize_jsonl(pred_path, "predictions")

    evaluate_predictions(pred_path, gt_path, metrics_path, include_meta=True)


if __name__ == "__main__":
    main()
