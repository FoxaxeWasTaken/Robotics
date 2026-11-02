#!/usr/bin/env bash
# Test of the release flow (#211) without network: milestone.sh, linked_milestones.sh (the PR check) and
# tools/release.sh. A fake `gh` answers each GET from a JSON file and logs the writes; a fake `curl` answers the
# SonarQube Cloud gate; release.sh runs in a throwaway clone of a throwaway "origin", so its tag push is real.
# pre-commit runs it when one of the scripts changes. Usage, from the repository root: bash tools/ci/test_release.sh
set -euo pipefail
src=$(pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*"; exit 1; }
export FAKE=$tmp/fake GH_LOG=$tmp/gh.log NOTES=$tmp/notes.md COMMENTS=$tmp/comments.json
mkdir -p "$FAKE" "$tmp/bin"
cat > "$tmp/bin/gh" <<'EOF'
#!/usr/bin/env bash
# A GET answers from $FAKE/<path, / ? & = turned into _>.json (the issue comments, read since a date, from
# $COMMENTS), or fails (not found); anything else is logged, its --notes-file copied to $NOTES.
if [ "$1" = api ] && [ "${2:-}" != -X ]; then
  [[ $2 == */comments* ]] && exec cat "$COMMENTS"
  file="$FAKE/$(tr '/?&=' '____' <<< "$2").json"
  [ -f "$file" ] && exec cat "$file"
  echo "gh: not found: $2" >&2; exit 1
fi
echo "gh $*" >> "$GH_LOG"
prev=""; for a in "$@"; do [ "$prev" = --notes-file ] && cp "$a" "$NOTES"; prev=$a; done
exit 0
EOF
cat > "$tmp/bin/curl" <<'EOF'
#!/usr/bin/env bash
echo "{\"projectStatus\": {\"status\": \"${GATE:-OK}\"}}"
EOF
chmod +x "$tmp/bin/gh" "$tmp/bin/curl"
export PATH="$tmp/bin:$PATH"
R=team/robotics
fake() { cat > "$FAKE/$(tr '/?&=' '____' <<< "$1").json"; }

fake "repos/$R/milestones?state=all&per_page=100" <<'EOF'
[{"number": 4, "title": "v1.0.0 — Sim", "state": "closed"}, {"number": 7, "title": "v1.2.0 — Camera", "state": "open"},
 {"number": 5, "title": "v1.1.0 — Tooling", "state": "open"}, {"number": 9, "title": "Someday", "state": "open"}]
EOF
fake "repos/$R/issues?milestone=5&state=open&per_page=100" <<'EOF'
[{"number": 212, "title": "[6.1d] Release v1.1.0 (real-arm tooling)", "labels": []},
 {"number": 9, "title": "[E6] Stage 6", "labels": [{"name": "epic"}]},
 {"number": 208, "title": "feat: a PR", "labels": [], "pull_request": {}}]
EOF
fake "repos/$R/issues?milestone=7&state=open&per_page=100" <<'EOF'
[{"number": 67, "title": "[6.2b] Intrinsics", "labels": [{"name": "task"}]}, {"number": 213, "title": "Release v1.2.0", "labels": []}]
EOF
ms=$src/tools/ci/milestone.sh
[ "$(bash "$ms" next $R)" = "$(printf '5\tv1.1.0 — Tooling\topen')" ] || fail "next: $(bash "$ms" next $R)"
[ "$(bash "$ms" find $R v1.2.0)" = "$(printf '7\tv1.2.0 — Camera\topen')" ] || fail "find: $(bash "$ms" find $R v1.2.0)"
[ -z "$(bash "$ms" find $R v9.0.0)" ] || fail "find made up a milestone"
[ -z "$(bash "$ms" blockers $R 5)" ] || fail "blockers counted the release ticket, an epic or a PR: $(bash "$ms" blockers $R 5)"
[ "$(bash "$ms" blockers $R 7)" = "#67" ] || fail "blockers: $(bash "$ms" blockers $R 7)"
echo "ok: milestone.sh: the oldest open version, a version's milestone, its open tickets but release tickets and epics"

# The PR check: every ticket a PR closes has a milestone.
for n in 69 200; do echo "{\"number\": $n, \"milestone\": {\"title\": \"v1.1.0 — Tooling\"}}" | fake "repos/$R/issues/$n"; done
echo '{"number": 77, "milestone": null}' | fake "repos/$R/issues/77"
check() { bash "$src/tools/ci/linked_milestones.sh" $R <<< "$1" > "$tmp/check.log" 2>&1; }
check $'## What\n\nCloses #69\nCloses #200\nSee #77, not closed here.' || fail "milestones set, refused: $(cat "$tmp/check.log")"
check $'Fixes #69\ncloses #77' && fail "#77 without a milestone, accepted"
grep -q "#77" "$tmp/check.log" || fail "the refusal does not name #77: $(cat "$tmp/check.log")"
check $'Resolves #404' && fail "an unreadable ticket, accepted"
check 'Bumps actions/setup-python from 7.0.0 to 7.1.0.' || fail "a PR that closes nothing, refused"
echo "ok: linked_milestones.sh: refuses a closed ticket without a milestone or unreadable, names it; nothing closed passes"

# release.sh in a throwaway clone: origin/main has the CHANGELOG section, the nightly bench reported on its commit.
git init -q --bare "$tmp/origin.git"
git clone -q "$tmp/origin.git" "$tmp/work" 2> /dev/null
cd "$tmp/work"
git config user.email t@t && git config user.name t
mkdir -p tools/ci && cp "$src/tools/release.sh" tools/ && cp "$ms" tools/ci/
printf '# Changelog\n\n## [Unreleased]\n\n## [1.1.0] - 2026-11-03\n\nThe tooling.\n\n### Added\n\n- tools\n\n## [1.0.0] - 2026-10-31\n\nOld.\n' > CHANGELOG.md
git add -A && git commit -qm init && git push -q origin HEAD:main 2> /dev/null
sha=$(git rev-parse HEAD)
report() {  # report <sha> <verdict>: the bench comment on the report issue, and that issue
  echo '[{"number": 155}]' | fake "repos/$R/issues?labels=report&state=all"
  python3 -c 'import json,sys; print(json.dumps([{"body": "### Nightly bench, 2026-11-03\n\n`main` at `%s`, 50 episodes\n\n**success 50/50**\n\nTarget ≥ 90 %%: %s.\n\nRun: https://github.com/team/robotics/actions/runs/77 (CSV)" % (sys.argv[1][:7], sys.argv[2])}]))' "$1" "$2" \
    > "$tmp/comments.json"
}
rel() { SONAR_TOKEN=${SONAR_TOKEN-x} REPO=$R bash tools/release.sh "$@" > "$tmp/rel.log" 2>&1; }
refused() {  # refused <expected message> -- <release.sh arguments>: exit 1, the message, nothing written
  local expected=$1; shift 2
  : > "$GH_LOG"
  rel "$@" && fail "release.sh $* accepted: $(cat "$tmp/rel.log")"
  grep -qF -- "$expected" "$tmp/rel.log" || fail "release.sh $*: no '$expected' in: $(cat "$tmp/rel.log")"
  [ ! -s "$GH_LOG" ] || fail "release.sh $* wrote: $(cat "$GH_LOG")"
  ! git ls-remote --tags origin | grep -q . || fail "release.sh $* pushed a tag"
}
report "$sha" met
refused "usage" -- 1.1.0
refused "no milestone v9.0.0" -- v9.0.0
refused "still has open tickets: #67" -- v1.2.0
refused "already closed" -- v1.0.0
SONAR_TOKEN="" refused "SONAR_TOKEN" -- v1.1.0
GATE=ERROR refused "quality gate of main: ERROR" -- v1.1.0
report 0000000000 met
refused "no nightly bench report on origin/main" -- v1.1.0
report "$sha" "not met"
refused "missed its target" -- v1.1.0
report "$sha" met
sed -i 's/## \[1.1.0\]/## [1.0.9]/' CHANGELOG.md && git commit -qam "no section" && git push -q origin HEAD:main 2> /dev/null
report "$(git rev-parse HEAD)" met
refused "no [1.1.0] section" -- v1.1.0
git revert --no-edit HEAD > /dev/null && git push -q origin HEAD:main 2> /dev/null
sha=$(git rev-parse HEAD)
report "$sha" met
echo "ok: release.sh refuses, with nothing written: a bad version, no or closed milestone, open tickets, no SONAR_TOKEN, a failed gate, no bench report on the commit, a missed target, no CHANGELOG section"

: > "$GH_LOG"
rel --dry-run v1.1.0 || fail "dry run: $(cat "$tmp/rel.log")"
grep -qF "+ git tag -a v1.1.0 $sha" "$tmp/rel.log" || fail "dry run, tag: $(cat "$tmp/rel.log")"
[ ! -s "$GH_LOG" ] && ! git ls-remote --tags origin | grep -q . || fail "a dry run wrote something"
rel v1.1.0 || fail "release: $(cat "$tmp/rel.log")"
[ "$(git ls-remote --tags origin "refs/tags/v1.1.0^{}" | cut -f1)" = "$sha" ] || fail "the tag is not on origin/main's commit"
grep -q "gh release create v1.1.0 -R $R --title v1.1.0 — Tooling --notes-file" "$GH_LOG" || fail "release: $(cat "$GH_LOG")"
grep -qF "The tooling." "$tmp/notes.md" && grep -qF "runs/77" "$tmp/notes.md" && ! grep -qF "Old." "$tmp/notes.md" \
  || fail "the notes are not the [1.1.0] section and the bench run: $(cat "$tmp/notes.md")"
grep -q "gh api -X PATCH repos/$R/milestones/5 -f state=closed" "$GH_LOG" || fail "milestone not closed: $(cat "$GH_LOG")"
! grep -q "milestones -f title=v1.2.0" "$GH_LOG" || fail "the next milestone made again"
grep -q "gh workflow run mirror.yml -R $R --ref main" "$GH_LOG" || fail "mirror not dispatched: $(cat "$GH_LOG")"
[ "$(grep -n "release create\|PATCH\|mirror.yml" "$GH_LOG" | cut -d: -f1 | tr '\n' ' ')" = "1 2 3 " ] || fail "order: $(cat "$GH_LOG")"
echo "ok: release.sh tags origin/main's commit, the Release from its CHANGELOG section and the bench run, closes the milestone, dispatches the mirror"
