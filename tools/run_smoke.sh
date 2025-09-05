#!/usr/bin/env bash
set -euo pipefail
echo "[smoke] repo: $(pwd)"

# 0) deps (Ensure dependencies are installed via uv)
uv pip install -q pandas motmetrics pytest pyyaml >/dev/null || true

# 1) fast evaluator on built-in scenario
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

# 3) MOT17-mini
echo "[smoke] eval-mot17"
if grep -q '^eval-mot17:' Makefile 2>/dev/null; then
  make clean-mot17 >/dev/null 2>&1 || true
  make eval-mot17
else
  echo "[smoke] (info) no eval-mot17 target found; skipping"
fi

# 4) KITTI-mini
echo "[smoke] eval-kitti"
make clean-kitti >/dev/null 2>&1 || true
make eval-kitti

# 5) pytest (unit + integration)
echo "[smoke] pytest"
pytest -q || { echo "[smoke] pytest failed"; exit 1; }

echo "[smoke] ALL GOOD ✅"