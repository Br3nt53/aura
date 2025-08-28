#!/usr/bin/env python3
import argparse, csv, json, os, glob
from pathlib import Path

def load_rows(csv_path):
    with open(csv_path, newline='') as f:
        return list(csv.DictReader(f))

def render_heatmap(rows, x, y, z, title):
    xs = sorted({r[x] for r in rows}, key=lambda v: float(v))
    ys = sorted({r[y] for r in rows}, key=lambda v: float(v))
    grid = [[float('nan')]*len(xs) for _ in ys]
    for r in rows:
        xi = xs.index(r[x]); yi = ys.index(r[y])
        try:
            grid[yi][xi] = float(r[z])
        except:
            pass
    import math
    def cell(v):
        s = "" if (isinstance(v, float) and math.isnan(v)) else f"{v:.3f}"
        zval = "NaN" if s=="" else s
        return f'<td data-z="{zval}">{s}</td>'
    def row(yv, arr):
        return "<tr><th>{}</th>{}</tr>".format(yv, "".join(cell(v) for v in arr))
    html = f"""<!doctype html><meta charset="utf-8"><title>{title}</title>
<style>body{{font-family:system-ui}}table{{border-collapse:collapse}}td,th{{border:1px solid #ddd;padding:6px}}</style>
<h2>{title}</h2><div>rows={y}, cols={x}, z={z}</div>
<table><tr><th>{y}\\{x}</th>{''.join(f'<th>{v}</th>' for v in xs)}</tr>
{''.join(row(ys[i], grid[i]) for i in range(len(ys)))}
</table>
<script>(function(){{
let t=document.querySelector('table'),zmin=1e9,zmax=-1e9;
let cells=[...t.querySelectorAll('td[data-z]')];
cells.forEach(td=>{{let v=parseFloat(td.dataset.z); if(!isNaN(v)){{zmin=Math.min(zmin,v); zmax=Math.max(zmax,v);}}}});
function color(v){{if(isNaN(v))return'#eee';let tt=(v-zmin)/(zmax-zmin+1e-9);let r=Math.round(255*(1-tt)),g=Math.round(255*tt),b=80;return`rgb(${r},${g},${b})`;}}
cells.forEach(td=>{{let v=parseFloat(td.dataset.z); td.style.background=color(v)}});
}})();</script>"""
    return html

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--experiments_dir', default='out/experiments')
    ap.add_argument('--out', default='out/report_bundle.html')
    args = ap.parse_args()

    exp_dirs = sorted(glob.glob(os.path.join(args.experiments_dir, '*', 'summary.csv')))
    htmls = []
    for csvp in exp_dirs:
        rows = load_rows(csvp)
        if not rows:
            continue
        keys = [k for k in rows[0].keys() if k not in ('auc','mota','id_switches','track_fragmentations','latency_ms')]
        if len(keys) < 2:
            continue
        heat = render_heatmap(rows, keys[0], keys[1], 'auc', f'Heatmap: {Path(csvp).parent.name}')
        htmls.append(heat)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text("<hr/>".join(htmls) or "<p>No experiments found.</p>")
    print('[report] wrote', args.out)

if __name__ == '__main__':
    main()
