#!/usr/bin/env python3
"""
Compute safe parameter bounds from experiment summaries:
- take runs within 95% of best AUC
- report min/max for each parameter across those runs
Writes JSON to unity/Assets/StreamingAssets/ParamBounds.json by default.
"""
import os, glob, csv, json, argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def collect():
    rows = []
    for p in glob.glob(str(ROOT / "out" / "experiments" / "*" / "summary.csv")):
        with open(p) as f:
            rows += list(csv.DictReader(f))
    return rows

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "unity" / "Assets" / "StreamingAssets" / "ParamBounds.json"))
    ap.add_argument("--keep_ratio", type=float, default=0.95)
    args = ap.parse_args()

    rows = collect()
    if not rows:
        bounds = {
            "rf_weight": {"min": 0.3, "max": 0.9},
            "track_decay_sec": {"min": 0.5, "max": 4.0},
            "assoc_gate_m": {"min": 0.8, "max": 2.5}
        }
    else:
        best = max(float(r.get("auc", 0.0)) for r in rows)
        kept = [r for r in rows if float(r.get("auc", 0.0)) >= args.keep_ratio * best]
        def range_for(k, default_min, default_max):
            vals = [float(r[k]) for r in kept if k in r and r[k] not in ("", None)]
            if not vals: return {"min": default_min, "max": default_max}
            return {"min": min(vals), "max": max(vals)}
        bounds = {
            "rf_weight": range_for("rf_weight", 0.3, 0.9),
            "track_decay_sec": range_for("track_decay_sec", 0.5, 4.0),
            "assoc_gate_m": range_for("assoc_gate_m", 0.8, 2.5)
        }
    outp = Path(args.out); outp.parent.mkdir(parents=True, exist_ok=True)
    json.dump(bounds, open(outp, "w"), indent=2)
    print("[bounds] wrote", outp)

if __name__ == "__main__":
    main()
