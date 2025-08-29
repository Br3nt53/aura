#!/usr/bin/env bash
set -euo pipefail
echo "[smoke] repo: $(pwd)"

# 0) venv + deps
command -v python3 >/dev/null || { echo "python3 not found"; exit 1; }
[ -x .venv/bin/python ] || python3 -m venv .venv
.venv/bin/python -m pip -q install --upgrade pip >/dev/null
.venv/bin/python -m pip -q install pandas motmetrics pytest pyyaml >/dev/null || true

# 1) fast evaluator on built-in scenario (already working on your machine)
echo "[smoke] eval-fast"
make clean-fast >/dev/null 2>&1 || true
make eval-fast

# 2) sample JSONL generator
echo "[smoke] eval-sample"
if grep -q '^eval-sample:' Makefile 2>/dev/null; then
  make clean-sample >/dev/null 2>&1 || true
  make eval-sample
else
  echo "[smoke] (info) no eval-sample target found; skipping"
fi

# 3) MOT17-mini (uses data/mot17-mini/gt.txt & det.txt you already created)
echo "[smoke] eval-mot17"
if grep -q '^eval-mot17:' Makefile 2>/dev/null; then
  make clean-mot17 >/dev/null 2>&1 || true
  make eval-mot17
else
  echo "[smoke] (info) no eval-mot17 target found; skipping"
fi

# 4) KITTI-mini (fixed flags)
echo "[smoke] eval-kitti"
make clean-kitti >/dev/null 2>&1 || true
make eval-kitti

# 5) pytest (unit + integration)
echo "[smoke] pytest"
.venv/bin/pytest -q || { echo "[smoke] pytest failed"; exit 1; }

echo "[smoke] ALL GOOD ✅"
