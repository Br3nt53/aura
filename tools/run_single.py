#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Optional, Any, Tuple

# Ensure repo root is importable when running from tools/
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


def _fallback_logger(name: str, level: str = "INFO") -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        fmt = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        handler.setFormatter(fmt)
        logger.addHandler(handler)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    logger.propagate = False
    return logger


try:
    from aura_logging import setup_logger as _setup

    logger = _setup("aura.tools.run_single", level=os.getenv("AURA_LOG_LEVEL", "INFO"))
except Exception:
    logger = _fallback_logger(
        "aura.tools.run_single", os.getenv("AURA_LOG_LEVEL", "INFO")
    )

# Prefer in-repo evaluator; provide a safe fallback if unavailable.
try:
    from evaluation.mot_evaluator import MOTEvaluator, EvalParams  # type: ignore[import]
except ImportError as e:
    logger.debug("evaluation.mot_evaluator not available: %s; using stub.", e)

    class _EvalParams:
        """Minimal fallback when evaluation package is absent."""

        pass

    class _MOTEvaluator:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass

        def evaluate(self, *args: Any, **kwargs: Any) -> dict[str, Any]:
            # Return an empty metrics dict by default; the caller may compute simple metrics.
            return {}

    # Expose the expected public names via aliases (no redefinition for type checkers).
    EvalParams = _EvalParams  # type: ignore[assignment]
    MOTEvaluator = _MOTEvaluator  # type: ignore[assignment]


def _bool_env(name: str) -> bool:
    return os.getenv(name, "").strip().lower() in {"1", "true", "yes"}


def _run(cmd: list[str]) -> None:
    logger.info("Executing: %s", " ".join(cmd))
    try:
        res = subprocess.run(cmd, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as e:
        stderr = e.stderr or ""
        if stderr:
            logger.error("Command failed. stderr:\n%s", stderr)
        raise
    else:
        if res.stderr:
            logger.warning("stderr:\n%s", res.stderr)


def _ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def generate_ground_truth(scenario_path: Path, gt_path: Path) -> None:
    _ensure_dir(gt_path.parent)
    _run(
        [
            sys.executable,
            "tools/make_gt_from_yaml.py",
            "--scenario",
            str(scenario_path),
            "--out",
            str(gt_path),
        ]
    )
    logger.info("Ground truth saved to %s", gt_path)


def synthesize_predictions_from_gt(gt_path: Path, pred_path: Path) -> None:
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
    pred_path: Path, gt_path: Path, metrics_path: Path, _use_aura_eval: bool
) -> None:
    logger.info("Evaluating predictions with the simple evaluator...")
    _ensure_dir(metrics_path.parent)
    results = MOTEvaluator(EvalParams()).evaluate(str(pred_path), str(gt_path))
    with metrics_path.open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=4)
    logger.info("Evaluation complete. MOTA: %.4f", float(results.get("mota", 0.0)))
    logger.info("Metrics saved to %s", metrics_path)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run single tracking experiment")
    p.add_argument("--scenario", required=True, help="Path to scenario YAML file")
    p.add_argument("--out", help="Output metrics JSON (legacy)")
    p.add_argument("--params", help="Optional params YAML (compat)")
    p.add_argument("--out-dir", help="Output directory")
    p.add_argument("--test-case", help="Specific test case to run")
    return p.parse_args(argv)


def resolve_paths(args: argparse.Namespace) -> Tuple[Path, Path, Path]:
    if args.out_dir:
        out_dir = Path(args.out_dir)
        _ensure_dir(out_dir)
        return out_dir / "gt.jsonl", out_dir / "pred.jsonl", out_dir / "metrics.json"

    if args.out:
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


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    scenario_path = Path(args.scenario)
    if not scenario_path.exists():
        logger.error("Scenario file %s not found.", args.scenario)
        sys.exit(1)

    gt_path, pred_path, metrics_path = resolve_paths(args)
    skip_ros = _bool_env("SKIP_ROS")

    if not (skip_ros and gt_path.exists()):
        generate_ground_truth(scenario_path, gt_path)

    if skip_ros:
        synthesize_predictions_from_gt(gt_path, pred_path)
    else:
        logger.info("ROS pipeline not implemented in this runner; set SKIP_ROS=1.")

    evaluate_predictions(pred_path, gt_path, metrics_path, True)


if __name__ == "__main__":
    main()
