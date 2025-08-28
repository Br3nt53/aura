#!/usr/bin/env python3
import argparse, json, os, subprocess, tempfile, time
from pathlib import Path
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args

def run_single(scenario, rf_weight, track_decay_sec, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = out_dir / "metrics.json"
    cmd = [
        "python3", "tools/run_single.py",
        "--scenario", scenario,
        "--rf_weight", str(rf_weight),
        "--wall_bonus", "0.4",                 # keep fixed; change if desired
        "--track_decay_sec", str(track_decay_sec),
        "--out", str(metrics_path)
    ]
    print("[exec]", " ".join(cmd))
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if r.returncode != 0:
        print(r.stdout)
        raise RuntimeError("run_single failed")
    try:
        m = json.load(open(metrics_path))
    except Exception as e:
        print("Failed to read metrics:", e)
        m = {"auc": 0.0}
    return m

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="scenarios/crossing_targets.yaml")
    ap.add_argument("--n_calls", type=int, default=25)
    ap.add_argument("--random_state", type=int, default=123)
    ap.add_argument("--outdir", default="out/bo_runs")
    args = ap.parse_args()

    outdir = Path(args.outdir); outdir.mkdir(parents=True, exist_ok=True)

    dim_rf_weight = Real(0.1, 1.0, name="rf_weight")
    dim_track_decay = Real(0.5, 5.0, name="track_decay_sec")
    dimensions = [dim_rf_weight, dim_track_decay]

    trials = []

    @use_named_args(dimensions=dimensions)
    def objective(rf_weight, track_decay_sec):
        run_id = len(trials)
        work = outdir / f"iter_{run_id:03d}"
        m = run_single(args.scenario, rf_weight, track_decay_sec, work)
        auc = float(m.get("auc", 0.0))
        trials.append({"iter": run_id, "rf_weight": rf_weight, "track_decay_sec": track_decay_sec, "auc": auc})
        # append log
        with open(outdir / "trials.jsonl", "a") as f:
            f.write(json.dumps(trials[-1]) + "\n")
        print(f"[result] iter={run_id} auc={auc:.4f}")
        # we minimize, so return negative AUC
        return -auc

    print("[bo] starting Bayesian optimization…")
    res = gp_minimize(objective, dimensions=dimensions, n_calls=args.n_calls, random_state=args.random_state)

    summary = {
        "best_auc": float(-res.fun),
        "best_params": {"rf_weight": float(res.x[0]), "track_decay_sec": float(res.x[1])},
        "n_calls": args.n_calls,
        "random_state": args.random_state,
        "scenario": args.scenario,
        "trials_count": len(trials)
    }
    json.dump(summary, open(outdir / "bo_summary.json", "w"), indent=2)
    print("[bo] done:", json.dumps(summary, indent=2))

if __name__ == "__main__":
    main()
