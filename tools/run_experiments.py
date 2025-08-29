#!/usr/bin/env python3
"""
Run experiment sweeps and aggregate results.

- Writes summary.csv with the sweep rows.
- Optionally writes an HTML heatmap using the same JS-safe technique as make_report.py.
"""

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List, Tuple


def _write_summary_csv(summary_path: Path, rows: List[Dict[str, str]]) -> None:
    keys = list(rows[0].keys())
    with open(summary_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def _build_grid(
    rows: List[Dict[str, str]], x: str, y: str, z: str
) -> Tuple[List[str], List[str], List[List[float]]]:
    xs = sorted(set(r[x] for r in rows))
    ys = sorted(set(r[y] for r in rows))
    grid = [[float("nan")] * len(xs) for _ in ys]

    for r in rows:
        xi = xs.index(r[x])
        yi = ys.index(r[y])
        try:
            grid[yi][xi] = float(r.get(z, float("nan")))
        except (TypeError, ValueError):
            pass

    return xs, ys, grid


def _render_html_heatmap(
    xs: List[str], ys: List[str], grid: List[List[float]], title: str
) -> str:
    data_json = json.dumps({"xs": xs, "ys": ys, "grid": grid})
    head = (
        "<!doctype html><meta charset='utf-8'>"
        "<title>" + title + "</title>"
        "<style>"
        "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;padding:24px}"
        "table{border-collapse:collapse}"
        "td,th{border:1px solid #ddd;padding:6px 10px;text-align:center}"
        "th{background:#fafafa}"
        ".num{font-variant-numeric:tabular-nums}"
        "</style>"
        "<h2>" + title + "</h2>"
    )
    js = """
<script>
(function () {
  const DATA = window.__AURA_DATA__;
  const xs = DATA.xs, ys = DATA.ys, grid = DATA.grid;

  const table = document.createElement('table');
  const thead = document.createElement('thead');
  const thr = document.createElement('tr');
  thr.appendChild(document.createElement('th'));
  xs.forEach(x => { const th = document.createElement('th'); th.textContent = x; thr.appendChild(th); });
  thead.appendChild(thr);
  table.appendChild(thead);

  const tbody = document.createElement('tbody');
  ys.forEach((y, yi) => {
    const tr = document.createElement('tr');
    const th = document.createElement('th');
    th.textContent = y;
    tr.appendChild(th);

    xs.forEach((x, xi) => {
      const td = document.createElement('td');
      const v = grid[yi][xi];
      td.dataset.z = String(v);
      td.className = 'num';
      td.textContent = isNaN(v) ? '–' : v.toFixed(3);
      tr.appendChild(td);
    });

    tbody.appendChild(tr);
  });
  table.appendChild(tbody);
  document.body.appendChild(table);

  const cells = Array.from(document.querySelectorAll('td[data-z]'));
  let zmin = Number.POSITIVE_INFINITY, zmax = Number.NEGATIVE_INFINITY;
  cells.forEach(td => { const v = parseFloat(td.dataset.z); if (!isNaN(v)) { zmin = Math.min(zmin, v); zmax = Math.max(zmax, v); } });

  function color(v) {
    if (isNaN(v)) return '#eee';
    const t = (v - zmin) / (zmax - zmin + 1e-9);
    const r = Math.round(255 * (1 - t));
    const g = Math.round(255 * t);
    const b = 80;
    return `rgb(${r},${g},${b})`;
  }
  cells.forEach(td => { const v = parseFloat(td.dataset.z); td.style.background = color(v); });
})();
</script>
"""
    data_script = "<script>window.__AURA_DATA__ = " + data_json + ";</script>"
    return head + data_script + js


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--outdir", type=Path, required=True, help="Directory to write outputs into."
    )
    ap.add_argument("--x", default="rf_weight", help="Field name for X axis.")
    ap.add_argument("--y", default="assoc_gate_m", help="Field name for Y axis.")
    ap.add_argument("--z", default="auc", help="Field name for Z values.")
    ap.add_argument("--html", action="store_true", help="Write an HTML heatmap report.")
    args = ap.parse_args()

    outdir = args.outdir
    outdir.mkdir(parents=True, exist_ok=True)

    # Example rows — replace with real sweep results in your pipeline
    rows: List[Dict[str, str]] = [
        {"rf_weight": "0.5", "assoc_gate_m": "1.0", "auc": "0.80"},
        {"rf_weight": "0.5", "assoc_gate_m": "1.2", "auc": "0.84"},
        {"rf_weight": "0.7", "assoc_gate_m": "1.0", "auc": "0.88"},
        {"rf_weight": "0.7", "assoc_gate_m": "1.2", "auc": "0.90"},
    ]

    # 1) CSV summary
    _write_summary_csv(outdir / "summary.csv", rows)

    # 2) Heatmap grid + optional HTML
    xs, ys, grid = _build_grid(rows, args.x, args.y, args.z)
    if args.html:
        html = _render_html_heatmap(
            xs, ys, grid, title=f"Heatmap — {args.z} vs {args.x}/{args.y}"
        )
        (outdir / "report.html").write_text(html, encoding="utf-8")


if __name__ == "__main__":
    main()
