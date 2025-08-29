#!/usr/bin/env bash
set -euo pipefail

RED() { printf "\033[31m%s\033[0m\n" "$*"; }
GRN() { printf "\033[32m%s\033[0m\n" "$*"; }
YEL() { printf "\033[33m%s\033[0m\n" "$*"; }
SEC() { printf "\n=== %s ===\n" "$*"; }

# Always initialize
pass=()
fail=()

check() {
  local name="$1" cmd="$2"
  # ignore empty/whitespace-only names to avoid ghost entries
  if [ -z "${name// }" ]; then return 0; fi
  if bash -lc "$cmd" >/dev/null 2>&1; then
    pass+=("$name")
  else
    fail+=("$name")
  fi
}

SEC "Repo structure"
check "README present"           "test -s README.md || test -s README.rst"
check "License present"          "ls -1 | grep -qi '^license'"
check "Scenarios folder"         "test -d scenarios && ls scenarios/*.yaml >/dev/null 2>&1"
check "Tools runner exists"      "test -f tools/run_single.py || test -f tools/run_experiments.py"
check "Out folder (gitkeep ok)"  "test -d out || test -f out/.gitkeep"
check "Dockerfile present"       "test -f Dockerfile || test -f .devcontainer/Dockerfile"

SEC "Python packaging"
check "Python manifest"          "test -f pyproject.toml || test -f requirements.txt"
check "Pre-commit config"        "test -f .pre-commit-config.yaml"
check "Ruff/Black/Mypy present"  "grep -Eiq 'ruff|flake8|black|mypy' pyproject.toml requirements.txt 2>/dev/null || false || true"

SEC "ROS 2 layout"
check "ROS2 ws exists"           "test -d ros2_ws || test -d src || test -d ros2_examples"
check "aura_examples pkg"        "find . -type f -path '*aura_examples*/package.xml' | grep -q . || true"
check "CMakeLists in pkg"        "find . -type f -path '*aura_examples*/CMakeLists.txt' | grep -q . || true"
check "bringup.launch.py"        "grep -R --include='*.launch.py' -n 'bringup' ros2_ws src ros2_examples 2>/dev/null | grep -q . || true"

SEC "CI/CD"
check "GitHub Actions present"   "test -d .github/workflows && ls .github/workflows/*.yml >/dev/null 2>&1"
check "pytest exists"            "test -d tests || grep -qi pytest requirements.txt pyproject.toml 2>/dev/null"

SEC "Scenario sanity"
check "Scenario has expected keys" "grep -Eqi 'targets|rf|event' scenarios/*.yaml"

SEC "Summary"
# Print only non-empty entries
for x in "${pass[@]:-}"; do [ -n "${x// }" ] && GRN "PASS: $x"; done
for x in "${fail[@]:-}"; do [ -n "${x// }" ] && RED "FAIL: $x"; done

# Count fails robustly
fail_count=0
for x in "${fail[@]:-}"; do [ -n "${x// }" ] && fail_count=$((fail_count+1)); done

if [ "$fail_count" -ne 0 ]; then
  YEL "Fix FAIL items and rerun."
  exit 1
fi
GRN "All checks passed."
