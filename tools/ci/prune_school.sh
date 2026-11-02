#!/usr/bin/env bash
# Deletes every branch of the school copy but main (#216, ADR-033 Update): nothing lives there, only main and the
# tags the mirror copies. Deleting a PR's head branch makes GitHub close that PR: Dependabot's (dependabot.yml is
# mirrored with main), a branch anyone pushed. The tags are left alone. Run by mirror.yml with the deploy key.
# Usage: bash tools/ci/prune_school.sh <remote>   (tested by test_prune_school.sh)
set -euo pipefail
remote=$1
branches=$(git ls-remote --heads "$remote" | cut -f2 | grep -vx refs/heads/main || true)
if [[ -z "$branches" ]]; then
  echo "Only main on $remote."
  exit 0
fi
for ref in $branches; do
  echo "Deleting ${ref#refs/heads/} (its PR, if any, closes)"
  git push -q "$remote" --delete "$ref"
done
