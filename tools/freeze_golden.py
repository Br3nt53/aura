#!/usr/bin/env python3
"""
Write config/golden_params.yaml from:
- out/bo_runs/bo_summary.json (preferred), or
- latest out/experiments/*/summary.csv best row.
Also write StreamingAssets/ParamBounds.json using derive_bounds.py, if available.
"""
import os, json, glob, csv, argparse, subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONF = ROOT / "config" / "golden_params.yaml"
STREAM = ROOT / "unity" / "Assets" / "StreamingAssets"
STREAM.mkdir(parents=True, exist_ok=True)

def best_from_sweeps():
    csvs = sorted(glob.glob(str(ROOT / "out" / "experiments" / "*" / "summary.csv")))
    best = None
    for c in csvs:
        with open(c) as f:
            r = list(csv.DictReader(f))
        for row in r:
            try:
                auc = float(row.get("auc", "nan"))
            except:
                continue
            if best is None or auc > best[0]:
                best = (auc, row)
    return best

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alpha", type=float, default=0.2, help="stability EMA alpha")
    ap.add_argument("--assoc_gate_m", type=float, default=None, help="override gate")
    args = ap.parse_args()

    params = {}
    bo = ROOT / "out" / "bo_runs" / "bo_summary.json"
    if bo.exists():
        d = json.load(open(bo))
        params["rf_weight"] = float(d["best_params"]["rf_weight"])
        params["track_decay_sec"] = float(d["best_params"]["track_decay_sec"])
    else:
        best = best_from_sweeps()
        if best:
            _, row = best
            for k in ("rf_weight","track_decay_sec"):
                if k in row:
                    params[k] = float(row[k])
    if args.assoc_gate_m is not None:
        params["assoc_gate_m"] = float(args.assoc_gate_m)
    params.setdefault("assoc_gate_m", 1.5)
    params["stability_alpha"] = float(args.alpha)
    params.setdefault("birth_stability_min", 0.35)
    params.setdefault("delete_stability_min", 0.15)

    CONF.parent.mkdir(parents=True, exist_ok=True)
    with open(CONF, "w") as f:
        for k,v in params.items():
            f.write(f"{k}: {v}\n")
    print("[golden] wrote", CONF)

    # Derive bounds if tool exists
    db = ROOT / "tools" / "derive_bounds.py"
    if db.exists():
        os.system(f"python3 {db} --out {STREAM/'ParamBounds.json'}")

if __name__ == "__main__":
    main()
