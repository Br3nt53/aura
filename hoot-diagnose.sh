#!/usr/bin/env bash
set -euo pipefail
echo "Shell: $SHELL"
echo "CWD: $PWD"
ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
echo "Git root: ${ROOT:-<none>}"
[ -n "${ROOT}" ] && cd "$ROOT" && echo "Now at git root: $(pwd)"
echo "Repo patches dir listing:"
ls -la patches || echo "(no ./patches directory in repo root)"
echo "Searching for *.patch (maxdepth 4):"
find . -maxdepth 4 -type f -name '*.patch' -print | sed 's#^\./##'
