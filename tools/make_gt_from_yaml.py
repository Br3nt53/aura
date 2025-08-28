#!/usr/bin/env python3
"""
Generate ground-truth JSONL from a scenario YAML.

Input YAML (examples in scenarios/*.yaml):
targets:
  - id: 1
    start: [x0, y0]
    end:   [x1, y1]
    speed_mps: 1.2  # optional; if omitted we linearly traverse over sim.duration_sec

sim:
  duration_sec: 16
  dt_sec: 0.05

Output JSONL (gt.jsonl):
{"frame": 0, "objects": [{"id": 1, "x": 2.0, "y": 10.0}, ...]}
{"frame": 1, "objects": [{"id": 1, "x": 2.06, "y": 10.0}, ...]}
...
"""
import argparse, json, math, sys
from pathlib import Path
import yaml

def linspace_xy(p0, p1, tfrac):
    x = p0[0] + (p1[0]-p0[0])*tfrac
    y = p0[1] + (p1[1]-p0[1])*tfrac
    return x, y

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True, help="Path to scenario YAML")
    ap.add_argument("--out", required=True, help="Output gt.jsonl path")
    args = ap.parse_args()

    data = yaml.safe_load(open(args.scenario, "r"))
    sim = data.get("sim", {})
    duration = float(sim.get("duration_sec", 10.0))
    dt = float(sim.get("dt_sec", 0.1))
    nframes = max(1, int(round(duration / dt)))

    targets = data.get("targets", [])
    # Precompute per-target total distance
    dists = []
    for t in targets:
        sx, sy = t["start"]
        ex, ey = t["end"]
        d = math.hypot(ex - sx, ey - sy)
        dists.append(d)

    # Build frames
    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)

    with outp.open("w") as f:
        for fi in range(nframes):
            tsec = fi * dt
            objs = []
            for t, dist in zip(targets, dists):
                tid = int(t["id"])
                sx, sy = t["start"]
                ex, ey = t["end"]
                speed = float(t.get("speed_mps", 0.0))

                # Compute fraction along path
                if speed > 0.0 and dist > 1e-6:
                    total_time = dist / speed
                    frac = min(1.0, tsec / max(1e-9, total_time))
                else:
                    # No speed specified: distribute evenly over total duration
                    frac = min(1.0, tsec / max(1e-9, duration))

                x, y = linspace_xy((sx,sy), (ex,ey), frac)
                objs.append({"id": tid, "x": x, "y": y})
            line = {"frame": fi, "objects": objs}
            f.write(json.dumps(line) + "\n")

if __name__ == "__main__":
    main()
