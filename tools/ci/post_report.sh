#!/usr/bin/env bash
# A nightly report (#151, #179): the Markdown file given, written to the run summary ($GITHUB_STEP_SUMMARY) and as
# one comment on the issue "Nightly bench results", created if none is open. Needs `gh` allowed to write issues
# (GH_TOKEN). Usage: bash tools/ci/post_report.sh <body.md>   (tested by test_bench_report.sh, test_quality_report.sh)
set -euo pipefail
title="Nightly bench results"
[[ -z ${GITHUB_STEP_SUMMARY:-} ]] || cat "$1" >> "$GITHUB_STEP_SUMMARY"
number=$(gh issue list --state open --limit 100 --json number,title --jq ".[] | select(.title == \"$title\") | .number" | head -n 1)
if [[ -z $number ]]; then
  intro=$(mktemp)
  echo "One comment per nightly bench and quality job (\`.github/workflows/nightly.yml\`, \`tools/ci/*_report.sh\`). Pin this issue." > "$intro"
  # label "report": the project views leave it out, it is not a work item
  number=$(gh issue create --title "$title" --label report --body-file "$intro" | sed -E 's|.*/issues/||')
fi
gh issue comment "$number" --body-file "$1"
echo "Commented on issue #$number"
