#!/usr/bin/env bash
# Nightly watch of the course repository (#137, docs/project/15 §2). When its main has moved past the commit
# recorded in .github/upstream-sha, opens one issue "Course repository: new commits" listing the new commits and
# the files they touch, or updates it if it is already open. Needs `gh` allowed to write issues (GH_TOKEN).
# Usage, from the repository root: bash tools/ci/upstream_check.sh [course_url]   (tested by test_upstream_check.sh)
set -euo pipefail
url=${1:-https://github.com/JasonChmn/content-lerobot-bras-students.git}
title="Course repository: new commits"
recorded=$(tr -d '[:space:]' < .github/upstream-sha)
head=$(git ls-remote "$url" refs/heads/main | cut -f1)
if [[ "$head" = "$recorded" ]]; then
  echo "The course repository is at the recorded commit ${recorded:0:7}: nothing to do"
  exit 0
fi
git fetch --quiet --filter=blob:none "$url" main  # commits and trees only: enough for the file names
body=$(mktemp)
{
  echo "The course repository (\`$url\`) has commits that \`main\` has not merged yet."
  if git cat-file -e "$recorded^{commit}" 2>/dev/null; then
    echo "Recorded in \`.github/upstream-sha\`: \`${recorded:0:7}\`; the course's \`main\`: \`${head:0:7}\`."
    range=("$recorded..FETCH_HEAD")
  else  # the course rewrote its history: show its latest commits
    echo "The recorded \`${recorded:0:7}\` is not in the course history any more; its latest commits:"
    range=(-20 FETCH_HEAD)
  fi
  echo
  echo '```text'
  git log --format='%h %ad %s' --date=short --name-status "${range[@]}"
  echo '```'
  echo
  echo "Merge them with the procedure of \`docs/project/15\` §2, which updates \`.github/upstream-sha\`, then close this issue."
} > "$body"
number=$(gh issue list --state open --limit 100 --json number,title --jq ".[] | select(.title == \"$title\") | .number" | head -n 1)
if [[ -n "$number" ]]; then
  gh issue edit "$number" --body-file "$body"
  echo "Updated issue #$number"
else
  gh issue create --title "$title" --body-file "$body"
fi
