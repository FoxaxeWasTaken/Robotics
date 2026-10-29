#!/usr/bin/env bash
# Nightly bench report (#151): the bench's summary line, the commit, the target and the run, written to the run
# summary ($GITHUB_STEP_SUMMARY) and as one comment on the issue "Nightly bench results", created if none is open.
# A bench that stopped before its summary line is reported too. Needs `gh` allowed to write issues (GH_TOKEN) and
# REF (the branch tested: main, or the branch of a workflow_dispatch), SHA, EPISODES, RUN_URL, ARTIFACT in the environment.
# Usage, from the repository root: bash tools/ci/bench_report.sh <bench log>   (tested by test_bench_report.sh)
set -euo pipefail
title="Nightly bench results"
target=90  # % over 50 episodes (00, 08 §2): informative, the nightly does not fail under it
line=$(grep -E '^success [0-9]+/[0-9]+ \([0-9]+ %\)' "$1" | tail -n 1 || true)
body=$(mktemp)
{
  echo "### Nightly bench, $(date -u +%F)"
  echo
  echo "\`$REF\` at \`${SHA:0:7}\`, $EPISODES episodes, randomized cube and box, real perception."
  echo
  if [[ -n "$line" ]]; then
    rate=$(sed -E 's/^success [0-9]+\/[0-9]+ \(([0-9]+) %\).*/\1/' <<< "$line")
    [[ "$rate" -ge "$target" ]] && verdict=met || verdict="not met"
    echo "**$line**"
    echo
    echo "Target ≥ $target %: $verdict."
  else
    echo "**The bench did not finish**: no summary line. See the run's log."
  fi
  echo
  echo "Run: $RUN_URL (CSV and the failed episodes' bags: artifact \`$ARTIFACT\`, kept 14 days)."
} > "$body"
[[ -z "${GITHUB_STEP_SUMMARY:-}" ]] || cat "$body" >> "$GITHUB_STEP_SUMMARY"
number=$(gh issue list --state open --limit 100 --json number,title --jq ".[] | select(.title == \"$title\") | .number" | head -n 1)
if [[ -z "$number" ]]; then
  intro=$(mktemp)
  echo "One comment per nightly bench (\`.github/workflows/nightly.yml\`, \`tools/ci/bench_report.sh\`). Pin this issue." > "$intro"
  # label "report": the project views leave it out, it is not a work item
  number=$(gh issue create --title "$title" --label report --body-file "$intro" | sed -E 's|.*/issues/||')
fi
gh issue comment "$number" --body-file "$body"
echo "Commented on issue #$number"
