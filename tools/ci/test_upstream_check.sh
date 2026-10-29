#!/usr/bin/env bash
# Test of upstream_check.sh (#137), without network or GitHub: a local bare repository stands in for the course
# and a fake `gh` on PATH records the issue calls. pre-commit runs it when either script changes.
# Usage, from the repository root: bash tools/ci/test_upstream_check.sh
set -euo pipefail
mapfile -t repo_vars < <(git rev-parse --local-env-vars)
unset "${repo_vars[@]}"  # as a git hook, GIT_INDEX_FILE & co. point at our repository, not the test's ones
check=$(pwd)/tools/ci/upstream_check.sh
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
export GIT_AUTHOR_NAME=test GIT_AUTHOR_EMAIL=test@example.com GIT_COMMITTER_NAME=test GIT_COMMITTER_EMAIL=test@example.com
fail() { echo "FAIL: $*"; exit 1; }

# The course: two commits on main, the second adds the J4 slides.
git init -q -b main "$tmp/work"
git -C "$tmp/work" commit -q --allow-empty -m "Add J3 pdf"
first=$(git -C "$tmp/work" rev-parse HEAD)
touch "$tmp/work/J4_visio.pdf" && git -C "$tmp/work" add J4_visio.pdf && git -C "$tmp/work" commit -q -m "Add J4 pdf"
git clone -q --bare "$tmp/work" "$tmp/course.git"

# Our repository and a fake gh: it logs each call; `issue list` prints $GH_OPEN, the open issue's number.
git init -q -b main "$tmp/team" && mkdir "$tmp/team/.github" "$tmp/bin"
cat > "$tmp/bin/gh" <<'EOF'
#!/usr/bin/env bash
echo "$*" >> "$GH_LOG"
[[ "$1 $2" = "issue list" ]] && printf '%s\n' "${GH_OPEN:-}"
prev=""; for a in "$@"; do [[ "$prev" = --body-file ]] && cp "$a" "$GH_LOG.body"; prev=$a; done
exit 0
EOF
chmod +x "$tmp/bin/gh"
export PATH="$tmp/bin:$PATH" GH_LOG="$tmp/gh.log"
run() {  # run <recorded sha> [open issue number]: the check, from our repository, against the local course
  : > "$GH_LOG" && rm -f "$GH_LOG.body"
  echo "$1" > "$tmp/team/.github/upstream-sha"
  (cd "$tmp/team" && GH_OPEN=${2:-} bash "$check" "$tmp/course.git") > "$tmp/out.log" 2>&1 || fail "exit $?: $(cat "$tmp/out.log")"
}

run "$(git -C "$tmp/work" rev-parse HEAD)"
[[ ! -s "$GH_LOG" ]] || fail "up to date, yet gh was called: $(cat "$GH_LOG")"
echo "ok: the recorded commit is the course's main: no issue"

run "$first"
grep -q '^issue create --title Course repository: new commits' "$GH_LOG" || fail "no issue opened: $(cat "$GH_LOG")"
grep -q "Add J4 pdf" "$GH_LOG.body" && grep -q "J4_visio.pdf" "$GH_LOG.body" || fail "body: $(cat "$GH_LOG.body")"
! grep -q "Add J3 pdf" "$GH_LOG.body" || fail "the recorded commit is listed as new"
echo "ok: a new course commit opens one issue listing it and its files"

run "$first" 7
grep -q '^issue edit 7 ' "$GH_LOG" && ! grep -q '^issue create' "$GH_LOG" || fail "not updated: $(cat "$GH_LOG")"
echo "ok: the issue already open is updated, not duplicated"

run 0123456789abcdef0123456789abcdef01234567
grep -q "not in the course history" "$GH_LOG.body" && grep -q "Add J4 pdf" "$GH_LOG.body" || fail "unknown: $(cat "$GH_LOG.body")"
echo "ok: a recorded commit the course no longer has is said so, with its latest commits"
