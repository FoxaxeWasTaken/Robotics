#!/usr/bin/env bash
# The PR check "Linked tickets have a milestone" (#211): fails when a ticket the PR closes (Closes, Fixes or
# Resolves #n in its body, on stdin) has no milestone, the version it ships in (ADR-028 Update, CONTRIBUTING §8),
# or cannot be read. A PR that closes nothing passes (Dependabot, chores). Needs gh (GH_TOKEN) and python3.
# Usage: bash tools/ci/linked_milestones.sh <owner/repo> < PR body   (tested by test_release.sh)
set -uo pipefail
repo=$1
numbers=$(grep -oiE '\b(close[sd]?|fix(e[sd])?|resolve[sd]?) #[0-9]+' | grep -oE '[0-9]+$' | sort -un)
if [[ -z "$numbers" ]]; then
  echo "This PR closes no ticket."
  exit 0
fi
missing=()
for n in $numbers; do
  if ! json=$(gh api "repos/$repo/issues/$n"); then
    echo "::error::#$n cannot be read"
    exit 1
  fi
  title=$(python3 -c 'import json, sys; m = json.load(sys.stdin)["milestone"]; print(m["title"] if m else "")' <<< "$json")
  if [[ -n "$title" ]]; then echo "#$n: $title"; else missing+=("#$n"); fi
done
if [[ ${#missing[@]} -gt 0 ]]; then
  echo "::error::No milestone on ${missing[*]}: set the version it ships in (CONTRIBUTING §8), then rerun this check"
  exit 1
fi
