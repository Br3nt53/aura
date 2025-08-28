#!/usr/bin/env python3
"""
Convert a CSV of detections into predictions JSONL.

CSV columns (header required):
frame,x,y,score,id

- "score" and "id" are optional; if missing, score=1.0 and id=0 used.
- Multiple detections per frame are allowed (rows with same frame).

Output JSONL:
{"frame": 0, "detections": [{"id": 0, "x": 2.0, "y": 10.0, "score": 0.9}]}
"""
import argparse, csv, json
from collections import defaultdict
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="Input CSV path")
    ap.add_argument("--out", required=True, help="Output pred.jsonl path")
    args = ap.parse_args()

    by_frame = defaultdict(list)
    with open(args.csv, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            frame = int(row["frame"])
            x = float(row["x"])
            y = float(row["y"])
            score = float(row.get("score", 1.0)) if row.get("score") not in (None, "") else 1.0
            pid = int(row.get("id", 0)) if row.get("id") not in (None, "") else 0
            by_frame[frame].append({"id": pid, "x": x, "y": y, "score": score})

    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    with outp.open("w") as f:
        for frame in sorted(by_frame.keys()):
            f.write(json.dumps({"frame": frame, "detections": by_frame[frame]}) + "\n")

if __name__ == "__main__":
    main()
