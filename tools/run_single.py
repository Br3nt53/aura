#!/usr/bin/env python3
import argparse, json, random, time, pathlib

# TODO: replace the mock section with your actual simulator + evaluator.

ap = argparse.ArgumentParser()
ap.add_argument("--scenario", required=True)
ap.add_argument("--rf_weight", type=float, required=True)
ap.add_argument("--wall_bonus", type=float, required=True)
ap.add_argument("--track_decay_sec", type=float, required=True)
ap.add_argument("--out", required=True)

    ap.add_argument("--ros2_pipeline", action="store_true", help="Use ROS2 launch pipeline to generate predictions")
    ap.add_argument("--workdir", help="Explicit workdir for pipeline and outputs (defaults to args.out directory)")
args = ap.parse_args()

# --- MOCKED METRICS (replace with real calls) ---
time.sleep(0.2)
seed = hash((args.scenario, args.rf_weight, args.wall_bonus, args.track_decay_sec)) & 0xffffffff
rnd = random.Random(seed)
auc = 0.6 + 0.35*(1.0 - abs(args.rf_weight-0.6)) - 0.05*abs(args.wall_bonus-0.4) - 0.03*abs(args.track_decay_sec-1.5) + 0.02*rnd.random()
mota = 0.5 + (auc - 0.5)*0.8
id_sw = int(10*(1.0-auc) + 2*rnd.random())
frag = int(6*(1.0-auc) + rnd.random())
lat_ms = int(35 + 20*(1.0-auc) + 5*rnd.random())
# -----------------------------------------------

pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
json.dump({
  "auc": max(0.0, min(1.0, auc)),
  "mota": max(0.0, min(1.0, mota)),
  "id_switches": id_sw,
  "track_fragmentations": frag,
  "latency_ms": lat_ms
}, open(args.out, "w"), indent=2)
