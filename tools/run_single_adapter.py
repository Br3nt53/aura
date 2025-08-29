#!/usr/bin/env python3
import os
import json
import argparse
import sys
from pathlib import Path

USE_REAL_EVALUATOR = os.environ.get("USE_REAL_EVALUATOR", "true").lower() in (
    "1",
    "true",
    "yes",
)
VALIDATE_EVALUATOR = os.environ.get("VALIDATE_EVALUATOR", "false").lower() in (
    "1",
    "true",
    "yes",
)


def parse_args():
    p = argparse.ArgumentParser(description="AURA run_single adapter (dual CLI)")
    # Original AURA contract
    p.add_argument("--scenario")
    p.add_argument("--rf_weight", type=float, default=0.0)
    p.add_argument("--wall_bonus", type=float, default=0.0)
    p.add_argument("--track_decay_sec", type=float, default=1.0)
    p.add_argument("--out")
    p.add_argument("--pred")
    p.add_argument("--gt")
    p.add_argument("--ros2_pipeline", action="store_true")
    p.add_argument("--workdir", default="out/tmp")
    # Current CLI
    p.add_argument("--params")
    p.add_argument("--out-dir")
    p.add_argument("--test-case")
    return p.parse_args()


def _derive_fps_from_env_and_yaml(scenario_path: str) -> float:
    fps = float(os.environ.get("AURA_FPS", "10.0"))
    try:
        import yaml  # type: ignore
    except ImportError:
        print(
            "[WARN] PyYAML not installed; using AURA_FPS or default 10.0",
            file=sys.stderr,
        )
        return fps
    try:
        if scenario_path and os.path.exists(scenario_path):
            with open(scenario_path, "r") as yf:
                doc = yaml.safe_load(yf) or {}
            fps_from_yaml = (
                (doc.get("sensor") or {}).get("fps")
                or (doc.get("camera") or {}).get("fps")
                or doc.get("fps")
            )
            if fps_from_yaml is not None:
                fps = float(fps_from_yaml)
    except Exception as e:
        print(
            f"[WARN] Failed to parse FPS from YAML {scenario_path}: {e}",
            file=sys.stderr,
        )
    return fps


def merge_to_stub_schema(stub_like: dict, real: dict) -> dict:
    out = {}
    for k in stub_like.keys():
        out[k] = real[k] if k in real else stub_like[k]
    for k, v in real.items():
        if k not in out:
            out[k] = v
    return out


def main():
    args = parse_args()
    Path(args.workdir).mkdir(parents=True, exist_ok=True)

    # Mode A: current runner (params/out-dir present)
    if args.params and args.out_dir:
        # Re-use existing tools/run_single.py to produce gt/pred/metrics (stub)
        # Then, if USE_REAL_EVALUATOR, compute real metrics and merge schema
        from tools import run_single as stub_runner  # type: ignore

        metrics_path = (
            stub_runner.run_stub_evaluation(
                args.scenario, args.params, args.out_dir, args.test_case
            )
            if hasattr(stub_runner, "run_stub_evaluation")
            else stub_runner.run_single_experiment(
                args.scenario, args.params, args.out_dir, args.test_case
            )
        )
        if USE_REAL_EVALUATOR:
            pred_file = str(Path(args.out_dir) / "pred.jsonl")
            gt_file = str(Path(args.out_dir) / "gt.jsonl")
            from evaluation.mot_evaluator import MOTEvaluator, EvalParams

            params = EvalParams(
                rf_weight=args.rf_weight,
                wall_bonus=args.wall_bonus,
                track_decay_sec=args.track_decay_sec,
                fps=_derive_fps_from_env_and_yaml(args.scenario),
                fragment_gap_mode=os.environ.get("AURA_FRAGMENT_GAP_MODE", "strict"),
            )
            real = MOTEvaluator(params).evaluate(
                pred_file=pred_file,
                gt_file=gt_file,
                scenario_path=args.scenario,
                workdir=args.out_dir,
                rf_weight=args.rf_weight,
                wall_bonus=args.wall_bonus,
                track_decay_sec=args.track_decay_sec,
            )
            try:
                with open(metrics_path, "r") as f:
                    stub_json = json.load(f)
            except Exception:
                stub_json = {}
            merged = merge_to_stub_schema(stub_json, real)
            if VALIDATE_EVALUATOR:
                with open(Path(args.out_dir) / "evaluator_diff.json", "w") as f:
                    json.dump({"stub": stub_json, "real": real}, f, indent=2)
            with open(metrics_path, "w") as f:
                json.dump(merged, f, indent=2)
        return

    # Mode B: original contract (pred/gt/out/workdir)
    if args.out is None:
        raise SystemExit("--out is required in original mode")
    pred_file = args.pred or str(Path(args.workdir) / "pred.jsonl")
    gt_file = args.gt or str(Path(args.workdir) / "gt.jsonl")
    if not (Path(pred_file).exists() and Path(gt_file).exists()):
        raise SystemExit(
            "pred/gt files are missing. Provide --pred/--gt or run your ROS2 pipeline first."
        )

    from evaluation.mot_evaluator import MOTEvaluator, EvalParams

    params = EvalParams(
        rf_weight=args.rf_weight,
        wall_bonus=args.wall_bonus,
        track_decay_sec=args.track_decay_sec,
        fps=_derive_fps_from_env_and_yaml(args.scenario),
        fragment_gap_mode=os.environ.get("AURA_FRAGMENT_GAP_MODE", "strict"),
    )
    real = MOTEvaluator(params).evaluate(
        pred_file=pred_file,
        gt_file=gt_file,
        scenario_path=args.scenario,
        workdir=args.workdir,
        rf_weight=args.rf_weight,
        wall_bonus=args.wall_bonus,
        track_decay_sec=args.track_decay_sec,
    )
    with open(args.out, "w") as f:
        json.dump(real, f, indent=2)


if __name__ == "__main__":
    main()
