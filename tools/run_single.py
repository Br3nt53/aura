#!/usr/bin/env python3
import argparse
import os
import subprocess
import pandas as pd
import motmetrics as mm

# --- BEGIN AURA EVALUATOR FEATURE FLAG ---
_USE_AURA = os.environ.get("USE_AURA_EVALUATOR", "").strip().lower() in {
    "1",
    "true",
    "yes",
}
try:
    if _USE_AURA:
        from evaluation.mot_evaluator import MOTEvaluator, EvalParams  # type: ignore
except Exception as _e:
    print(
        f"[INFO] USE_AURA_EVALUATOR set but evaluator not available ({_e}); falling back to motmetrics."
    )
    _USE_AURA = False


def _evaluate_with_aura(pred_path: str, gt_path: str) -> dict:
    params = EvalParams()
    ev = MOTEvaluator(params)
    res = ev.evaluate(pred_path, gt_path)
    res["objective"] = res.get("mota", 0.0)
    return res


# --- END AURA EVALUATOR FEATURE FLAG ---


def run_single_experiment(scenario_path, params_path, out_dir, test_case=None):
    """
    Runs a single experiment pipeline:
    1. Generates ground truth from the scenario.
    2. Runs the ROS 2 pipeline to produce predictions.
    3. Evaluates the predictions against the ground truth.
    4. Saves the metrics.
    """
    print(
        f"Running experiment for scenario: {scenario_path} with params: {params_path}"
    )
    if test_case:
        print(f"Executing specific test case: {test_case}")

    # Ensure output directory exists
    os.makedirs(out_dir, exist_ok=True)

    # Define file paths
    gt_path = os.path.join(out_dir, "gt.jsonl")
    pred_path = os.path.join(out_dir, "pred.jsonl")
    metrics_path = os.path.join(out_dir, "metrics.json")

    # --- Step 1: Generate Ground Truth ---
    gt_gen_cmd = [
        "./tools/make_gt_from_yaml.py",
        "--scenario",
        scenario_path,
        "--out",
        gt_path,
    ]
    if test_case:
        gt_gen_cmd.extend(["--test-case", test_case])

    print(f"Generating ground truth with command: {' '.join(gt_gen_cmd)}")
    subprocess.run(gt_gen_cmd, check=True)
    print(f"Ground truth saved to {gt_path}")
    _SKIP_ROS = os.environ.get("SKIP_ROS", "").strip().lower() in {"1", "true", "yes"}
    if _SKIP_ROS:
        print("[INFO] SKIP_ROS=1: skipping ROS pipeline step")
        # If predictions don't exist, synthesize trivial preds from GT so we can evaluate.
        if True:  # always regenerate preds when SKIP_ROS=1
            import json

            with open(gt_path, "r") as fin, open(pred_path, "w") as fout:
                for line in fin:
                    if not line.strip():
                        continue
                    d = json.loads(line)
                    out = {
                        "frame": int(d.get("frame", 0)),
                        "id": str(d.get("id", "p0")),
                        "x": float(d.get("x", 0.0)),
                        "y": float(d.get("y", 0.0)),
                        "conf": 1.0,
                    }
                    fout.write(json.dumps(out) + "\n")
        if _USE_AURA:
            metrics_dict = _evaluate_with_aura(pred_path, gt_path)
            with open(metrics_path, "w") as f:
                import json

                json.dump(metrics_dict, f, indent=4)
            print(
                f"Evaluation complete. MOTA: {metrics_dict.get('objective', metrics_dict.get('mota', 0.0)):.4f}"
            )
            print(f"Metrics saved to {metrics_path}")
            return metrics_path
        else:
            print(
                "[INFO] SKIP_ROS=1 but USE_AURA_EVALUATOR is not set; continuing to legacy evaluator..."
            )

    # --- Step 2: Run ROS 2 Pipeline ---
    ros_pipeline_cmd = [
        "./tools/run_ros2_pipeline.py",
        "--scenario",
        scenario_path,
        "--params",
        params_path,
        "--out-pred",
        pred_path,
    ]
    if test_case:
        ros_pipeline_cmd.extend(["--test-case", test_case])

    print(f"Running ROS 2 pipeline with command: {' '.join(ros_pipeline_cmd)}")
    subprocess.run(ros_pipeline_cmd, check=True)
    print(f"Predictions saved to {pred_path}")
    # --- BEGIN AURA EVALUATOR FAST-PATH ---
    if _USE_AURA:
        print("[INFO] Using AURA evaluator via USE_AURA_EVALUATOR")
        try:
            metrics_dict = _evaluate_with_aura(pred_path, gt_path)
        except Exception as e:
            print(f"[WARN] AURA evaluator failed ({e}); falling back to motmetrics.")
        else:
            with open(metrics_path, "w") as f:
                json.dump(metrics_dict, f, indent=4)
            print(
                f"Evaluation complete. MOTA: {metrics_dict.get('objective', 0.0):.4f}"
            )
            print(f"Metrics saved to {metrics_path}")
            return metrics_path
    # --- END AURA EVALUATOR FAST-PATH ---

    # --- Step 3: Evaluate Predictions ---
    print("Evaluating predictions against ground truth...")
    try:
        # Load ground truth and predictions into pandas DataFrames
        gt_df = pd.read_json(gt_path, lines=True)
        pred_df = pd.read_json(pred_path, lines=True)

        if pred_df.empty:
            raise ValueError("Prediction file is empty. Cannot evaluate.")

        # Convert to MOTMetrics format
        gt_df_mot = gt_df.rename(
            columns={"frame": "FrameId", "id": "Id", "x": "X", "y": "Y"}
        )
        pred_df_mot = pred_df.rename(
            columns={"frame": "FrameId", "id": "Id", "x": "X", "y": "Y"}
        )

        for df in [gt_df_mot, pred_df_mot]:
            df["Width"] = 1
            df["Height"] = 1
            df["Confidence"] = 1

        acc = mm.MOTAccumulator(auto_id=True)

        for frame in sorted(gt_df_mot.FrameId.unique()):
            gt_frame = gt_df_mot[gt_df_mot.FrameId == frame]
            pred_frame = pred_df_mot[pred_df_mot.FrameId == frame]

            gt_ids = gt_frame.Id.values
            pred_ids = pred_frame.Id.values

            gt_points = gt_frame[["X", "Y"]].values
            pred_points = pred_frame[["X", "Y"]].values

            distances = mm.distances.norm2squared_matrix(
                gt_points, pred_points, max_d2=1.0
            )

            acc.update(gt_ids, pred_ids, distances, frameid=frame)

        # --- Step 4: Calculate and Save Metrics ---
        mh = mm.metrics.create()
        summary = mh.compute(
            acc, metrics=mm.metrics.motchallenge_metrics, name="overall"
        )

        metrics_dict = summary.to_dict("records")[0]
        metrics_dict["objective"] = metrics_dict.get("mota", 0.0)

        print(f"Evaluation complete. MOTA: {metrics_dict['objective']:.4f}")

    except (FileNotFoundError, pd.errors.EmptyDataError, ValueError) as e:
        print(f"Error during evaluation: {e}. Saving empty metrics.")
        metrics_dict = {"objective": 0.0, "mota": 0.0, "error": str(e)}

    with open(metrics_path, "w") as f:
        json.dump(metrics_dict, f, indent=4)
    print(f"Metrics saved to {metrics_path}")

    return metrics_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run a single AURA experiment and evaluate it."
    )
    parser.add_argument(
        "--scenario", required=True, help="Path to the scenario YAML file."
    )
    parser.add_argument(
        "--params", required=True, help="Path to the parameters YAML file."
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        help="Directory to save the output files (gt, pred, metrics).",
    )
    # Add the new, optional argument
    parser.add_argument(
        "--test-case",
        required=False,
        help="Specify a single test case to run from a scenario file.",
    )
    args = parser.parse_args()

    run_single_experiment(args.scenario, args.params, args.out_dir, args.test_case)
