#!/usr/bin/env bash
# Prints heavy=true when a PR run needs the build, the tests and the E2E job, heavy=false otherwise (#146, #209).
# $1 is the PR's draft state when the run executes (gh pr view --json isDraft), never the event's: a push while
# draft and a "ready" in the same second start two runs, and the concurrency group cancels one of them, maybe the
# ready one; a rerun keeps its event too. A draft runs lint and guards only; anything but "true" counts as ready,
# so a failed lookup runs everything. Ready: everything, unless the changed paths (stdin) are documentation only.
# Usage: git diff --name-only --no-renames <base> <head> | bash tools/ci/heavy.sh <true|false>
set -uo pipefail
if [ "${1:-}" = true ]; then
  echo "Draft: lint and guards only; Ready for review runs every job." >&2
  echo heavy=false
elif bash "$(dirname "$0")/docs_only.sh" >&2; then
  echo heavy=false
else
  echo heavy=true
fi
