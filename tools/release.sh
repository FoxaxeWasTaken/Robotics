#!/usr/bin/env bash
# release.sh - cut a version (docs/project/15 §6, ADR-028 Update, #211), from the repository root, gh logged in and
# SONAR_TOKEN set (SonarQube Cloud: My Account → Security). Refuses, with nothing written, unless:
#   - the version's milestone is open, with no open ticket but its release ticket and the epics;
#   - the nightly bench reported on origin/main's exact commit, its target met (issue "Nightly bench results");
#   - the SonarQube Cloud quality gate of main is passed;
#   - CHANGELOG.md on origin/main has the version's section (its PR merged first: main is protected).
# Then: the annotated tag on that commit, the GitHub Release (the CHANGELOG section and the bench run), the milestone
# closed and the next minor's opened if missing, the mirror dispatched. It never pushes to the school repository:
# the mirror does (docs/project/16). --dry-run prints the steps. Tested by tools/ci/test_release.sh.
# Usage: tools/release.sh [--dry-run] vX.Y.Z
set -euo pipefail
REPO=${REPO:-FoxaxeWasTaken/Robotics}
SONAR_PROJECT=FoxaxeWasTaken_Robotics
ms="$(dirname "$0")/ci/milestone.sh"
dry=false
die() { echo "release: $*" >&2; exit 1; }
run() { echo "+ $*"; "$dry" || "$@"; }

[[ "${1:-}" = --dry-run ]] && { dry=true; shift; }
[[ ${1:-} =~ ^v([0-9]+)\.([0-9]+)\.([0-9]+)$ ]] || die "usage: tools/release.sh [--dry-run] vX.Y.Z"
version=$1 major=${BASH_REMATCH[1]} minor=${BASH_REMATCH[2]}

IFS=$'\t' read -r number title state <<< "$(bash "$ms" find "$REPO" "$version")" || true
[[ -n "${number:-}" ]] || die "no milestone $version (ADR-028 Update): create it, with its tickets"
[[ "$state" = open ]] || die "the milestone $title is already closed"
blockers=$(bash "$ms" blockers "$REPO" "$number" | tr '\n' ' ')
[[ -z "$blockers" ]] || die "the milestone $title still has open tickets: $blockers(close them or move them to the next version)"

git fetch -q origin main --tags
sha=$(git rev-parse origin/main)
! git rev-parse -q --verify "refs/tags/$version" > /dev/null || die "the tag $version exists"

[[ -n "${SONAR_TOKEN:-}" ]] || die "SONAR_TOKEN is not set: the quality gate of main cannot be checked"
gate=$(curl -fsS --max-time 30 -K - "https://sonarcloud.io/api/qualitygates/project_status?projectKey=$SONAR_PROJECT&branch=main" \
  <<< "header = \"Authorization: Bearer $SONAR_TOKEN\"" | python3 -c 'import json, sys; print(json.load(sys.stdin)["projectStatus"]["status"])') \
  || die "SonarQube Cloud did not answer"
[[ "$gate" = OK ]] || die "the quality gate of main: $gate"

report=$(gh api "repos/$REPO/issues?labels=report&state=all" | python3 -c 'import json, sys; print(json.load(sys.stdin)[0]["number"])') \
  || die "no issue labeled report (Nightly bench results)"
since=$(date -u -d '-30 days' +%FT%TZ)
bench=$(gh api "repos/$REPO/issues/$report/comments?since=$since&per_page=100" | python3 -c '
import json, re, sys
sha = sys.argv[1]
for c in reversed(json.load(sys.stdin)):
    body = c["body"]
    if body.startswith("### Nightly bench") and "at `%s`" % sha[:7] in body:
        met = "met" if re.search(r"Target .*: met\.", body) else "missed"
        run = re.search(r"Run: (\S+)", body)
        print(met, run.group(1) if run else "")
        break
' "$sha")
[[ -n "$bench" ]] || die "no nightly bench report on origin/main ($sha): gh workflow run nightly.yml -R $REPO --ref main, then again once its report is posted"
read -r verdict bench_run <<< "$bench"
[[ "$verdict" = met ]] || die "the nightly bench on $sha missed its target: $bench_run"

notes=$(mktemp)
git show "$sha:CHANGELOG.md" | awk -v v="## [${version#v}]" 'index($0, v) == 1 {on = 1; next} on && /^## \[/ {exit} on' > "$notes"
grep -q '[^[:space:]]' "$notes" || die "CHANGELOG.md on origin/main has no [${version#v}] section: merge its PR first"
printf '\nThe nightly bench of this commit: %s\n' "$bench_run" >> "$notes"
"$dry" && { echo "Notes:"; cat "$notes"; }

run git tag -a "$version" "$sha" -m "$title"
run git push -q origin "$version"
run gh release create "$version" -R "$REPO" --title "$title" --notes-file "$notes" --verify-tag
run gh api -X PATCH "repos/$REPO/milestones/$number" -f state=closed
next="v$major.$((minor + 1)).0"
[[ -n "$(bash "$ms" find "$REPO" "$next")" ]] || run gh api -X POST "repos/$REPO/milestones" -f title="$next"
run gh workflow run mirror.yml -R "$REPO" --ref main
echo "Then: the mirror's run, git ls-remote --tags <school url> lists $version (docs/project/16), the release ticket closed with the links."
