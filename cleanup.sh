#!/usr/bin/env bash
set -euo pipefail

say() { printf '%b\n' "$*"; }
SEC() { say "\n=== $* ==="; }

SEC "Start"
branch="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')"
say "[cleanup] On branch: ${branch}"

# 1) Ensure .gitignore protects the workspace & junk
SEC "Normalizing .gitignore"
gitignore=.gitignore
touch "$gitignore"

ensure_line() {
  grep -qxF "$1" "$gitignore" || echo "$1" >> "$gitignore"
}

# macOS & Python cruft
ensure_line ".DS_Store"
ensure_line "__pycache__/"
ensure_line ".pytest_cache/"
ensure_line ".mypy_cache/"
ensure_line ".venv/"
ensure_line ".venv*/"

# ROS 2 workspace artifacts (do not track anything in ros2_ws)
ensure_line "ros2_ws/"
# Unity caches (if present)
ensure_line "unity/Library/"
ensure_line "unity/Temp/"
ensure_line "unity/Logs/"
ensure_line "unity/Obj/"
# Colcon metadata
ensure_line ".colcon/"
ensure_line "log/"

# 2) Make ros2_ws/src/aura_examples a local symlink to the canonical source (untracked)
SEC "Setting up local symlink (untracked) to canonical ROS package"
mkdir -p ros2_ws/src
if [ -e ros2_ws/src/aura_examples ] && [ ! -L ros2_ws/src/aura_examples ]; then
  rm -rf ros2_ws/src/aura_examples
fi
if [ ! -L ros2_ws/src/aura_examples ]; then
  ln -s ../../ros2_examples/aura_examples ros2_ws/src/aura_examples
  say "[cleanup] Symlink created: ros2_ws/src/aura_examples -> ../../ros2_examples/aura_examples"
else
  say "[cleanup] Symlink already in place."
fi

# 3) Stop tracking any workspace/build/junk already in the index (keep files locally)
SEC "Purging tracked artifacts from the index (kept locally)"
# Anything under ros2_ws/* should be untracked
git ls-files -z "ros2_ws" | xargs -0r git rm -r --cached --quiet || true
# mac junk accidentally staged
git ls-files -z | grep -z "/\.DS_Store$" | xargs -0r git rm --cached --quiet || true
# Python caches accidentally staged
git ls-files -z | grep -z "/__pycache__/" | xargs -0r git rm -r --cached --quiet || true

# 4) Ensure pre-commit is available and run once (handles Ruff/Black)
SEC "Running pre-commit hooks"
if ! command -v pre-commit >/dev/null 2>&1; then
  python3 -m pip install -U pre-commit ruff black
fi
pre-commit install
# Stash-aware double run helps avoid “stashed changes conflicted with hook auto-fixes”
pre-commit run --all-files || true
pre-commit run --all-files

SEC "Done"
say "Next:"
say "  git add -A"
say "  git commit -m \"chore: cleanup workspace; single-source ROS pkg; tooling\""
say "  git pull --rebase origin main   # resolve any conflicts"
say "  git push origin main"
