#!/usr/bin/env bash
# Nightly bench report (#151): the bench's summary line, the commit, the target and the run, posted by post_report.sh
# (run summary, one comment on the issue "Nightly bench results"). A bench that stopped before its summary line is
# reported too. Needs `gh` allowed to write issues (GH_TOKEN) and REF (the branch tested: main, or the branch of a workflow_dispatch), SHA, EPISODES, RUN_URL, ARTIFACT in the environment.
# Usage, from the repository root: bash tools/ci/bench_report.sh <bench log>   (tested by test_bench_report.sh)
set -euo pipefail
target=90  # % over 50 episodes (00, 08 §2): informative, the nightly does not fail under it
line=$(grep -E '^success [0-9]+/[0-9]+ \([0-9]+ %\)' "$1" | tail -n 1 || true)
body=$(mktemp)
{
  echo "### Nightly bench, $(date -u +%F)"
  echo
  echo "\`$REF\` at \`${SHA:0:7}\`, $EPISODES episodes, randomized cube and box, real perception."
  echo
  if [ -n "$line" ]; then
    rate=$(sed -E 's/^success [0-9]+\/[0-9]+ \(([0-9]+) %\).*/\1/' <<< "$line")
    [ "$rate" -ge "$target" ] && verdict=met || verdict="not met"
    echo "**$line**"
    echo
    echo "Target ≥ $target %: $verdict."
  else
    echo "**The bench did not finish**: no summary line. See the run's log."
  fi
  echo
  echo "Run: $RUN_URL (CSV and the failed episodes' bags: artifact \`$ARTIFACT\`, kept 14 days)."
} > "$body"
bash "$(dirname "${BASH_SOURCE[0]}")/post_report.sh" "$body"
