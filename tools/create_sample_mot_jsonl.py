#!/usr/bin/env python3
"""
Convert MOTChallenge txt files to JSONL schema.
- GT: data_root/gt/gt.txt
- Pred: data_root/det/det.txt (if missing, synthesize from GT)
"""

import argparse
import json
import pathlib


def read_mot_txt(p: pathlib.Path):
    rows = []
    for line in p.read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split(",")
        frame = int(float(parts[0]))
        tid = int(float(parts[1]))
        x = float(parts[2])
        y = float(parts[3])
        w = float(parts[4])
        h = float(parts[5])
        rows.append({"frame": frame, "id": tid, "x": x, "y": y, "w": w, "h": h})
    return rows


def write_jsonl(path: pathlib.Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    root = pathlib.Path(args.data_root)
    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    gt_txt = root / "gt" / "gt.txt"
    det_txt = root / "det" / "det.txt"

    gt_rows = read_mot_txt(gt_txt)

    pr_rows = []
    if det_txt.exists():
        for line in det_txt.read_text().splitlines():
            if not line.strip():
                continue
            parts = line.split(",")
            frame = int(float(parts[0]))
            pid = f"p{int(float(parts[1]))}"
            x = float(parts[2])
            y = float(parts[3])
            w = float(parts[4])
            h = float(parts[5])
            pr_rows.append(
                {"frame": frame, "id": pid, "x": x, "y": y, "w": w, "h": h, "conf": 1.0}
            )
    else:
        pr_rows = [{**r, "id": f"p{r['id']}", "conf": 1.0} for r in gt_rows]

    write_jsonl(out / "gt.jsonl", gt_rows)
    write_jsonl(out / "pred.jsonl", pr_rows)

    print(f"[mot] wrote {len(gt_rows)} gt and {len(pr_rows)} preds to {out}")


if __name__ == "__main__":
    main()
