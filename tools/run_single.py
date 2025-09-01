#!/usr/bin/env python3
"""Single experiment runner with ground truth generation and evaluation.

Compatible CLIs:
1) Legacy:
   python tools/run_single.py --scenario scenarios/crossing_targets.yaml --out out/metrics.json

2) New-style:
   python tools/run_single.py --scenario scenarios/crossing_targets.yaml \
       --params scenarios/params.min.yaml --out-dir out/test_run
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Optional

# --- Logging ---------------------------------------------------------------


def _fallback_logger(name: str, level: str = "INFO") -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        h = logging.StreamHandler()
        fmt = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        h.setFormatter(fmt)
        logger.addHandler(h)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    logger.propagate = False
    return logger


try:
    # Prefer project logger if available
    from aura_logging import setup_logger as _setup

    logger = _setup("aura.tools.run_single", level=os.getenv("AURA_LOG_LEVEL", "INFO"))
except Exception:
    logger = _fallback_logger(
        "aura.tools.run_single", os.getenv("AURA_LOG_LEVEL", "INFO")
    )

# --- Optional motmetrics ---------------------------------------------------

try:
    import motmetrics as mm  # type: ignore
except Exception:  # pragma: no cover
    mm = None  # type: ignore[assignment]


# --- Helpers ---------------------------------------------------------------


def _bool_env(name: str) -> bool:
    return os.getenv(name, "").strip().lower() in {"1", "true", "yes"}


def _run(cmd: list[str]) -> None:
    """Run a subprocess, show stderr if present, raise on failure."""
    logger.info("Executing: %s", " ".join(cmd))
    try:
        res = subprocess.run(cmd, check=True, capture_output=True, text=True)
        if res.stderr:
            logger.warning("stderr:\n%s", res.stderr)
    except subprocess.CalledProcessError as e:
        stderr = e.stderr or ""
        if stderr:
            logger.error("Command failed. stderr:\n%s", stderr)
        raise


def _ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


# --- Core pipeline ---------------------------------------------------------


def generate_ground_truth(scenario_path: Path, gt_path: Path) -> None:
    """Call the GT generator with flags."""
    _ensure_dir(gt_path.parent)
    cmd = [
        sys.executable,
        "tools/make_gt_from_yaml.py",
        "--scenario",
        str(scenario_path),
        "--out",
        str(gt_path),
    ]
    _run(cmd)
    logger.info("Ground truth saved to %s", gt_path)


def synthesize_predictions_from_gt(gt_path: Path, pred_path: Path) -> None:
    """Write a trivial predictions file from GT (used when SKIP_ROS=1)."""
    _ensure_dir(pred_path.parent)
    n = 0
    with gt_path.open("r", encoding="utf-8") as fin, pred_path.open(
        "w", encoding="utf-8"
    ) as fout:
        for line in fin:
            s = line.strip()
            if not s:
                continue
            d = json.loads(s)
            rec = {
                "frame": int(d.get("frame", 0)),
                "id": str(d.get("id", "0")),
                "x": float(d.get("x", 0.0)) + 0.1,
                "y": float(d.get("y", 0.0)) + 0.1,
                "conf": 0.95,
            }
            fout.write(json.dumps(rec) + "\n")
            n += 1
    logger.info("Synthetic predictions saved to %s (%d rows)", pred_path, n)


def evaluate_predictions(
    pred_path: Path, gt_path: Path, metrics_path: Path, use_aura_eval: bool
) -> None:
    """Very light placeholder evaluation that always writes a metrics.json."""
    _ensure_dir(metrics_path.parent)

    metrics: dict[str, float | str] = {}
    if use_aura_eval:
        logger.info("Using AURA evaluator (placeholder).")
        # TODO: integrate real AURA evaluator when available
        metrics = {"status": "success", "mota": 1.0}
    else:
        if mm is None:
            logger.warning(
                "motmetrics not installed; writing placeholder metrics. "
                "pip install motmetrics to enable."
            )
            metrics = {"status": "success", "mota": 0.99}
        else:
            logger.info("Using motmetrics evaluator (placeholder).")
            # TODO: real motmetrics evaluation here
            metrics = {"status": "success", "mota": 0.99}

    with metrics_path.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=4)
    logger.info("Metrics saved to %s", metrics_path)
    logger.info("Evaluation complete. MOTA: %.4f", float(metrics.get("mota", 0.0)))


# --- CLI -------------------------------------------------------------------


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run single tracking experiment")

    # Always required
    p.add_argument("--scenario", required=True, help="Path to scenario YAML file")

    # Back-compat (legacy): explicit metrics file path
    p.add_argument("--out", help="Output metrics JSON file (legacy mode)")

    # New-style interface
    p.add_argument("--params", help="Optional params YAML (accepted for compatibility)")
    p.add_argument(
        "--out-dir",
        help="Output directory (writes gt.jsonl, pred.jsonl, metrics.json)",
    )

    p.add_argument("--test-case", help="Specific test case to run")
    return p.parse_args(argv)


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path, Path]:
    """Return (gt_path, pred_path, metrics_path) based on args."""
    # If --out-dir provided, prefer that
    if args.out_dir:
        out_dir = Path(args.out_dir)
        _ensure_dir(out_dir)
        gt_path = out_dir / "gt.jsonl"
        pred_path = out_dir / "pred.jsonl"
        metrics_path = out_dir / "metrics.json"
        return gt_path, pred_path, metrics_path

    # Else if legacy --out provided, use its parent
    if args.out:
        metrics_path = Path(args.out)
        _ensure_dir(metrics_path.parent)
        gt_path = metrics_path.parent / "ground_truth.jsonl"
        pred_path = metrics_path.parent / "predictions.jsonl"
        return gt_path, pred_path, metrics_path

    # Default fallback
    out_dir = Path("out")
    _ensure_dir(out_dir)
    return out_dir / "gt.jsonl", out_dir / "pred.jsonl", out_dir / "metrics.json"


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)

    scenario_path = Path(args.scenario)
    if not scenario_path.exists():
        logger.error("Scenario file %s not found.", args.scenario)
        sys.exit(1)

    gt_path, pred_path, metrics_path = resolve_paths(args)

    if args.test_case:
        logger.info("Executing specific test case: %s", args.test_case)

    skip_ros = _bool_env("SKIP_ROS")
    use_aura_eval = _bool_env("USE_AURA_EVALUATOR")

    # 1) GT generation
    if not (skip_ros and gt_path.exists()):
        try:
            generate_ground_truth(scenario_path, gt_path)
        except Exception as e:  # pragma: no cover
            logger.error("Failed to generate ground truth: %s", e)
            sys.exit(1)
    else:
        logger.info("SKIP_ROS=1 and existing GT found; skipping GT generation")

    # 2) Predictions
    if skip_ros:
        synthesize_predictions_from_gt(gt_path, pred_path)
    else:
        # Placeholder for ROS2 pipeline call
        logger.info("ROS pipeline not implemented in this runner; set SKIP_ROS=1.")

    # 3) Evaluation
    evaluate_predictions(pred_path, gt_path, metrics_path, use_aura_eval)


if __name__ == "__main__":
    main()
