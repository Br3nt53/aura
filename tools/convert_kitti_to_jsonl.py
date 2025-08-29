#!/usr/bin/env python3
import argparse
import json
import pathlib

"""
Reads a single KITTI-Tracking label file and writes JSONL with:
frame, id, x, y, w, h  (x,y = left-top)
KITTI tracking label columns:
frame, type, truncated, occluded, alpha, left, top, right, bottom, h3d, w3d, l3d, X, Y, Z, ry, score (optional)
track id is appended as the last column in newer splits OR provided separately; in the canonical format used by training,
track id is column 1 after frame (see devkit variants). To be robust, we parse the official tracking label format:
frame, id, type, truncated, occluded, alpha, left, top, right, bottom, ... (many)
If your files are the older format without id in col 2, use a converter or your devkit to inject ids first.
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--labels",
        required=True,
        help="path to one KITTI tracking label file (e.g. label_02/0000.txt)",
    )
    ap.add_argument("--out", required=True, help="output jsonl file")
    ap.add_argument(
        "--types",
        default="",
        help="csv of types to keep (e.g. Pedestrian,Car). Empty=keep all.",
    )
    args = ap.parse_args()

    keep = {t.strip() for t in args.types.split(",")} if args.types else None
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    n_in = n_out = 0
    with open(args.labels, "r") as fin, open(out, "w") as fout:
        for line in fin:
            line = line.strip()
            if not line:
                continue
            n_in += 1
            parts = line.split()

            # Two common variants:
            # Variant A (common in KITTI-tracking): frame, id, type, truncated, occluded, alpha, left, top, right, bottom, ...
            # Variant B (older): frame, type, truncated, occluded, alpha, left, top, right, bottom, ... (NO id)
            try:
                frame = int(parts[0])
                # Heuristic: if second token is an integer -> it's the id; else it's the type (older format)
                try:
                    tid = int(parts[1])
                    typ = parts[2]
                    off = 3
                except ValueError:
                    tid = -1  # unknown
                    typ = parts[1]
                    off = 2

                left = float(parts[off + 4])
                top = float(parts[off + 5])
                right = float(parts[off + 6])
                bot = float(parts[off + 7])
            except Exception:
                continue

            if keep is not None and typ not in keep:
                continue

            w = max(0.0, right - left)
            h = max(0.0, bot - top)
            rec = {"frame": frame, "id": tid, "x": left, "y": top, "w": w, "h": h}
            fout.write(json.dumps(rec) + "\n")
            n_out += 1

    print(f"[convert] read {n_in} lines, wrote {n_out} rows -> {out}")


if __name__ == "__main__":
    main()
