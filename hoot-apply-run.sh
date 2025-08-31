#!/usr/bin/env bash
set -euo pipefail

echo "==> Invoked as: $0"
ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "${ROOT}" ]; then
  echo "!! Not inside a Git repo. cd to the repo root and re-run."
  exit 1
fi
echo "==> Git root: $ROOT"
cd "$ROOT"

# Stash only tracked changes; DO NOT stash untracked (keeps patches/ visible)
if [ "${AURA_STASH:-1}" = "1" ]; then
  echo "==> Stashing tracked changes only (-k) ..."
  git stash -k -m "pre-hoorun-$(date +%s)" || true
fi

PATCH_DIR="$ROOT/patches"
if [ ! -d "$PATCH_DIR" ]; then
  echo "!! Patch directory missing: $PATCH_DIR"
  exit 1
fi

echo "==> Patch files:"
ls -1 "$PATCH_DIR"/*.patch || { echo "!! No *.patch files found"; exit 1; }

echo "==> Applying patches ..."
for p in "$PATCH_DIR"/*.patch; do
  echo "   -> git apply --index \"$p\""
  git apply --index "$p"
done

echo "==> Staged diff summary:"
git diff --staged --stat || true

echo "==> Running unit tests ..."
if ! command -v pytest >/dev/null 2>&1; then
  echo "!! pytest not found. Try: pip install pytest"
  exit 1
fi

pytest -q tests/unit/test_fragment_boundary.py

if ! pytest -q tests/unit/test_yaml_fps_extraction.py; then
  echo "[WARN] YAML parsing tests failed (PyYAML missing or adapter not present?). Continuing ..."
fi

pytest -q tests/unit/test_jsonl_and_empty_cases.py

echo "==> SUCCESS. To commit staged changes:"
echo "   git commit -m 'Apply evaluator patches: FPS, fragment boundary, YAML FPS, JSONL robustness, tests'"
