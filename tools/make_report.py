#!/usr/bin/env python3
"""
Generate experiment heatmap reports (JSON and optional HTML).

- Reads a CSV summary (e.g., summary.csv).
- Builds a z-grid for (x, y) axes.
- Can emit JSON to stdout or write an HTML heatmap if --html-out is provided.

Usage:
  python tools/make_report.py summary.csv rf_weight assoc_gate_m auc --html-out out/report.html
"""

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List


def build_grid(rows: List[Dict[str, str]], x: str, y: str, z: str):
    xs = sorted(set(r[x] for r in rows))
    ys = sorted(set(r[y] for r in rows))
    grid = [[float("nan")] * len(xs) for _ in ys]

    for r in rows:
        xi = xs.index(r[x])
        yi = ys.index(r[y])
        try:
            grid[yi][xi] = float(r[z])
        except Exception:
            # skip non-numeric cells
            pass

    return xs, ys, grid


def render_html_heatmap(
    xs: List[str], ys: List[str], grid: List[List[float]], title: str
) -> str:
    """
    Return a self-contained HTML string with a simple heatmap table.
    NOTE: We avoid Python f-strings around the JavaScript to prevent Ruff F821
    from interpreting `${...}` inside JS template literals.
    """
    # Inject data as JSON via concatenation (no f-strings around JS).
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

    # Plain JS (no Python formatting). We load `DATA` as a global var via <script> below.
    js = """
<script>
(function () {
  const DATA = window.__AURA_DATA__;
  const xs = DATA.xs, ys = DATA.ys, grid = DATA.grid;

  // Build table
  const table = document.createElement('table');
  const thead = document.createElement('thead');
  const thr = document.createElement('tr');
  thr.appendChild(document.createElement('th')); // corner
  xs.forEach(x => {
    const th = document.createElement('th');
    th.textContent = x;
    thr.appendChild(th);
  });
  thead.appendChild(thr);
  table.appendChild(thead);

  const tbody = document.createElement('tbody');
  ys.forEach((y, yi) => {
    const tr = document.createElement('tr');
    const rowHeader = document.createElement('th');
    rowHeader.textContent = y;
    tr.appendChild(rowHeader);

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

  // Colorize cells based on min/max
  const cells = Array.from(document.querySelectorAll('td[data-z]'));
  let zmin = Number.POSITIVE_INFINITY, zmax = Number.NEGATIVE_INFINITY;
  cells.forEach(td => {
    const v = parseFloat(td.dataset.z);
    if (!isNaN(v)) {
      zmin = Math.min(zmin, v);
      zmax = Math.max(zmax, v);
    }
  });

  function color(v) {
    if (isNaN(v)) return '#eee';
    const t = (v - zmin) / (zmax - zmin + 1e-9);
    // green-to-red (you can adjust if you prefer)
    const r = Math.round(255 * (1 - t));
    const g = Math.round(255 * t);
    const b = 80;
    return `rgb(${r},${g},${b})`;
  }

  cells.forEach(td => {
    const v = parseFloat(td.dataset.z);
    td.style.background = color(v);
  });
})();
</script>
"""

    data_script = "<script>window.__AURA_DATA__ = " + data_json + ";</script>"
    return head + data_script + js


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("x")
    ap.add_argument("y")
    ap.add_argument("z")
    ap.add_argument(
        "--html-out", type=Path, help="Optional path to write an HTML heatmap."
    )
    args = ap.parse_args()

    with open(args.csv) as f:
        rows = list(csv.DictReader(f))

    xs, ys, grid = build_grid(rows, args.x, args.y, args.z)

    # Always print JSON (useful for pipelines)
    json.dump(
        {"xs": xs, "ys": ys, "grid": grid},
        fp=sys.stdout if (sys := __import__("sys")) else None,
        indent=2,
    )

    # Optionally write HTML
    if args.html_out:
        args.html_out.parent.mkdir(parents=True, exist_ok=True)
        html = render_html_heatmap(
            xs, ys, grid, title=f"Heatmap — {args.z} vs {args.x}/{args.y}"
        )
        args.html_out.write_text(html, encoding="utf-8")


if __name__ == "__main__":
    main()
