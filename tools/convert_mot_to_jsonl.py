#!/usr/bin/env python3
import json
import pathlib
import random


def main(out_dir="out/sample-mot"):
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    gt = out / "gt.jsonl"
    pr = out / "pred.jsonl"

    random.seed(0)
    W, H = 1280, 720

    def box(cx, cy, w=40, h=80):
        x = max(0.0, min(W - w, cx - w / 2))
        y = max(0.0, min(H - h, cy - h / 2))
        return x, y, w, h

    g, p = [], []
    for t in range(30):
        objs = {
            1: (200 + 5 * t, 300),
            2: (600, 150 + 3 * t),
            3: (900 - 4 * t, 500 - 2 * t),
        }
        for tid, (cx, cy) in objs.items():
            x, y, w, h = box(cx, cy)
            g.append({"frame": t, "id": tid, "x": x, "y": y, "w": w, "h": h})

            if t not in {7, 18}:
                jx = x + random.uniform(-3, 3)
                jy = y + random.uniform(-3, 3)
                p.append(
                    {
                        "frame": t,
                        "id": f"p{tid}",
                        "x": jx,
                        "y": jy,
                        "w": w,
                        "h": h,
                        "conf": 1.0,
                    }
                )

    gt.write_text("\n".join(json.dumps(r) for r in g), encoding="utf-8")
    pr.write_text("\n".join(json.dumps(r) for r in p), encoding="utf-8")

    print(f"[sample] wrote {len(g)} GT -> {gt}")
    print(f"[sample] wrote {len(p)} Pred -> {pr}")


if __name__ == "__main__":
    main()
