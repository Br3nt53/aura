#!/usr/bin/env python3
import argparse, json, itertools, os, subprocess, time, math, statistics
from pathlib import Path

def ensure_dir(p: Path):
    p.mkdir(parents=True, exist_ok=True)

def grid(d):
    keys = list(d.keys())
    vals = [d[k] for k in keys]
    for combo in itertools.product(*vals):
        yield dict(zip(keys, combo))

def run_once(cmd_list, timeout):
    t0 = time.time()
    try:
        p = subprocess.run(cmd_list, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=timeout, check=False, text=True)
        return dict(
            rc=p.returncode, secs=time.time()-t0,
            out=p.stdout, err=p.stderr
        )
    except subprocess.TimeoutExpired:
        return dict(rc=-1, secs=time.time()-t0, out="", err="TIMEOUT")

def median_or_nan(xs):
    xs = [x for x in xs if isinstance(x,(int,float)) and not math.isnan(x)]
    return statistics.median(xs) if xs else float("nan")

def load_metrics(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return {}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    args = ap.parse_args()
    cfg = json.load(open(args.config))
    exp_name = cfg["experiment_name"]
    out_dir = Path(cfg["output_dir"])
    ensure_dir(out_dir)

    combos = list(grid(cfg["sweep"]))
    print(f"[exp] {exp_name}: {len(combos)} grid points × {cfg.get('repetitions',1)} reps")

    records = []
    for i, params in enumerate(combos, 1):
        for rep in range(cfg.get("repetitions",1)):
            run_dir = out_dir / f"run_{i:04d}_rep{rep}"
            ensure_dir(run_dir)
            cmd = [x.format(run_dir=str(run_dir), scenario=cfg["scenario"], **params)
                   for x in cfg["runner"]["cmd"]]
            print(f"[run] {i:04d} rep{rep} :: {' '.join(cmd)}")
            r = run_once(cmd, cfg["runner"].get("timeout_sec", 900))
            metrics = load_metrics(run_dir / "metrics.json")
            rec = dict(params, rep=rep, rc=r["rc"], secs=r["secs"], **metrics)
            records.append(rec)

    # aggregate by combo (median across reps)
    keys = list(cfg["sweep"].keys())
    agg = {}
    for rec in records:
        key = tuple(rec[k] for k in keys)
        agg.setdefault(key, []).append(rec)
    rows = []
    for key, lst in agg.items():
        merged = {k:v for k,v in zip(keys, key)}
        for m in ["auc","mota","id_switches","track_fragmentations","latency_ms"]:
            merged[m] = median_or_nan([x.get(m,float("nan")) for x in lst])
        rows.append(merged)

    # write CSV + HTML
    import csv
    csv_path = out_dir / "summary.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys+["auc","mota","id_switches","track_fragmentations","latency_ms"])
        w.writeheader()
        for r in rows: w.writerow(r)

    html = render_html_heatmap(rows, keys, x=keys[0], y=keys[1], z="auc", title=f"{exp_name} — AUC")
    (out_dir / "report.html").write_text(html)
    print(f"[done] {csv_path}")
    print(f"[done] {out_dir/'report.html'}")

def render_html_heatmap(rows, keys, x, y, z, title):
    import math as _math
    # map to grid
    xs = sorted(sorted({r[x] for r in rows}, key=lambda v: float(v)))
    ys = sorted(sorted({r[y] for r in rows}, key=lambda v: float(v)))
    grid = [[float('nan')]*len(xs) for _ in ys]
    for r in rows:
        xi = xs.index(r[x]); yi = ys.index(r[y])
        grid[yi][xi] = r.get(z, float('nan'))
    def _row(yval, arr):
        def cell(v):
            s = "" if (isinstance(v, float) and _math.isnan(v)) else f"{v:.3f}"
            z = "NaN" if s=="" else s
            return f'<td data-z="{z}">{s}</td>'
        return f"<tr><th>{yval}</th>{''.join(cell(v) for v in arr)}</tr>"
    return f"""<!doctype html>
<html><head><meta charset="utf-8"><title>{title}</title>
<style>
body{{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial}}
table{{border-collapse:collapse;margin:16px 0}}
td,th{{border:1px solid #ddd;padding:6px 8px;text-align:center}}
.caption{{opacity:.8}}
</style></head><body>
<h2>{title}</h2>
<div class="caption">Z = {z}; rows = {y}; cols = {x}</div>
<table>
<tr><th>{y}\\{x}</th>{"".join(f"<th>{v}</th>" for v in xs)}</tr>
{"".join(_row(ys[i], grid[i]) for i in range(len(ys)))}
</table>
<script>
(function(){{
  const t=document.querySelector('table');
  let zmin=Infinity,zmax=-Infinity;
  const cells=[...t.querySelectorAll('td[data-z]')];
  cells.forEach(td=>{{const v=parseFloat(td.dataset.z); if(!isNaN(v)){{zmin=Math.min(zmin,v); zmax=Math.max(zmax,v);}}}});
  function color(v){{
    if(isNaN(v)) return '#eee';
    const t=(v - zmin) / (zmax - zmin + 1e-9);
    const r=Math.round(255*(1-t)), g=Math.round(255*(t)), b=80;
    return `rgb(${r},${g},${b})`;
  }}
  cells.forEach(td=>{{const v=parseFloat(td.dataset.z); td.style.background=color(v)}});
}})();
</script>
</body></html>"""

if __name__ == "__main__":
    main()
