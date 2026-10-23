#!/usr/bin/env bash
# Nightly (#146): is the full evaluation bench due? Prints evaluate=true|false, for $GITHUB_OUTPUT.
# Not due when the newest nightly run whose bench gave a verdict (success or failure) ran on this commit of main:
# nothing changed since. Any doubt (no such run among the last 30, gh failing) means due: a skip never hides an
# untested main. Needs GH_TOKEN (actions: read) and GH_REPO.
# Usage: bash tools/ci/evaluation_due.sh <sha of main>
set -uo pipefail
sha=$1
due() { echo "$1" >&2; echo "evaluate=true"; exit 0; }
runs=$(gh api "repos/$GH_REPO/actions/workflows/nightly.yml/runs?branch=main&status=completed&per_page=30" \
  --jq '.workflow_runs[] | "\(.id) \(.head_sha)"') || due "Nightly runs not listed: the bench runs."
while read -r id head; do
  [ -n "$id" ] || continue
  verdict=$(gh api "repos/$GH_REPO/actions/runs/$id/jobs" \
    --jq '.jobs[] | select(.name == "Full evaluation bench") | .conclusion') || due "Run $id not read: the bench runs."
  case $verdict in
    success | failure)
      [ "$head" = "$sha" ] || due "Last bench: run $id on $head; main is now $sha: the bench runs."
      echo "The bench already ran on $sha (run $id, $verdict): skipped." >&2
      echo "evaluate=false"
      exit 0 ;;
  esac
done <<< "$runs"
due "No bench verdict among the last 30 nightly runs: the bench runs."
