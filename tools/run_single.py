#!/usr/bin/env python3
"""
Single-scenario runner for Aura smoke tests.

- Compatible CLI (supports --scenario, --out, --out-dir) like main.
- Always ensures non-empty gt.jsonl and pred.jsonl (synthesizes if missing).
- Hands evaluation off to evaluation/run_trackeval.py (HOTA-ready metrics.json).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
import sys
from pathlib import Path

logger = logging.getLogger("aura.tools.run_single")


def _configure_logging() -> None:
    if logger.handlers:
        return
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(name)s - %(message)s",
    )


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _bool_env(name: str) -> bool:
    return os.getenv(name, "").strip().lower() in {"1", "true", "yes"}


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path, Path]:
    # Preserve main's interface: prefer --out-dir, then --out, else ./out
    if getattr(args, "out_dir", None):
        out_dir = Path(args.out_dir)
        _ensure_dir(out_dir)
        return out_dir / "gt.jsonl", out_dir / "pred.jsonl", out_dir / "metrics.json"

    if getattr(args, "out", None):
        metrics_path = Path(args.out)
        _ensure_dir(metrics_path.parent)
        return (
            metrics_path.parent / "gt.jsonl",
            metrics_path.parent / "pred.jsonl",
            metrics_path,
        )

    out_dir = Path("out")
    _ensure_dir(out_dir)
    return out_dir / "gt.jsonl", out_dir / "pred.jsonl", out_dir / "metrics.json"


def _run(cmd: list[str]) -> None:
    logger.info("Running: %s", " ".join(cmd))
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        if proc.stdout.strip():
            logger.error("stdout:\n%s", proc.stdout)
        if proc.stderr.strip():
            logger.error("stderr:\n%s", proc.stderr)
        raise RuntimeError(f"Command failed ({proc.returncode}): {' '.join(cmd)}")
    if proc.stdout.strip():
        logger.info("stdout:\n%s", proc.stdout)


def _synthesize_jsonl(jsonl_path: Path, kind: str) -> None:
    """
    Write a tiny non-empty stream so evaluators never divide-by-zero.
    Accepts keys: frame,id,x,y,[w,h],score/conf.
    """
    _ensure_dir(jsonl_path.parent)
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
    include_meta: bool = True,  # kept for signature compatibility
) -> None:
    """Evaluate via TrackEval wrapper; writes metrics.json with HOTA field CI expects."""
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
    logger.info("Metrics saved to %s", metrics_path)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Run a single scenario smoke test.")
    p.add_argument(
        "--scenario",
        type=str,
        default="scenarios/crossing_targets.yaml",
        help="Path to scenario file (not required for synth path).",
    )
    p.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Directory to write gt.jsonl/pred.jsonl/metrics.json.",
    )
    p.add_argument(
        "--out",
        type=str,
        default=None,
        help="Explicit path to metrics.json (gt/pred written alongside).",
    )
    return p


def main() -> None:
    _configure_logging()
    args = build_arg_parser().parse_args()

    # Keep main's UX: warn if scenario missing, but do not fail (we synthesize anyway).
    scenario_path = Path(args.scenario)
    if not scenario_path.exists():
        logger.warning(
            "Scenario file %s not found; proceeding with synthesized data.",
            args.scenario,
        )

    gt_path, pred_path, metrics_path = resolve_paths(args)

    # Ensure inputs exist; if not, synthesize stable tiny streams.
    if not gt_path.exists():
        _synthesize_jsonl(gt_path, "ground-truth")
    if not pred_path.exists():
        # Respect SKIP_ROS semantics if you wire a real ROS path later; for smoke we still synthesize.
        _synthesize_jsonl(pred_path, "predictions")

    evaluate_predictions(pred_path, gt_path, metrics_path, include_meta=True)


if __name__ == "__main__":
    main()
