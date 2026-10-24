#!/usr/bin/env bash
# Test of bench_report.sh (#151), without network or GitHub: a fake `gh` on PATH records the issue calls.
# pre-commit runs it when either script changes. Usage, from the repository root: bash tools/ci/test_bench_report.sh
set -euo pipefail
report=$(pwd)/tools/ci/bench_report.sh
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*"; exit 1; }

# Fake gh: it logs each call; `issue list` prints $GH_OPEN, the open issue's number; `issue create` prints the new
# issue's URL; the --body-file of a call is copied to $GH_LOG.body.
mkdir "$tmp/bin"
cat > "$tmp/bin/gh" <<'EOF'
#!/usr/bin/env bash
echo "$*" >> "$GH_LOG"
[ "$1 $2" = "issue list" ] && printf '%s\n' "${GH_OPEN:-}"
[ "$1 $2" = "issue create" ] && echo "https://github.com/team/robotics/issues/42"
prev=""; for a in "$@"; do [ "$prev" = --body-file ] && cp "$a" "$GH_LOG.body"; prev=$a; done
exit 0
EOF
chmod +x "$tmp/bin/gh"
export PATH="$tmp/bin:$PATH" GH_LOG="$tmp/gh.log" GH_REPO=team/robotics REF=main SHA=0123456789abcdef EPISODES=50 \
  RUN_URL=https://github.com/team/robotics/actions/runs/7 ARTIFACT=evaluation-3

run() {  # run <bench log> [open issue number]: the report, with a step summary file
  : > "$GH_LOG" && rm -f "$GH_LOG.body" && : > "$tmp/summary.md"
  printf '%s\n' "$1" > "$tmp/bench.log"
  GITHUB_STEP_SUMMARY="$tmp/summary.md" GH_OPEN=${2:-} bash "$report" "$tmp/bench.log" > "$tmp/out.log" 2>&1 ||
    fail "exit $?: $(cat "$tmp/out.log")"
}
FULL="episode 50: DONE, 15.1 s, success
success 46/50 (92 %); duration p50 15.2 s, p95 17.4 s; causes: object_lost: 4"

run "$FULL" 12
grep -q '^issue comment 12 --body-file' "$GH_LOG" && ! grep -q '^issue create' "$GH_LOG" || fail "$(cat "$GH_LOG")"
for e in "success 46/50 (92 %)" "p95 17.4 s" "object_lost: 4" "\`main\` at \`0123456\`" "50 episodes" "Target ≥ 90 %: met." \
  "$RUN_URL" "evaluation-3"; do
  grep -qF "$e" "$GH_LOG.body" || fail "no '$e' in the comment: $(cat "$GH_LOG.body")"
done
cmp -s "$GH_LOG.body" "$tmp/summary.md" || fail "the run summary differs from the comment: $(cat "$tmp/summary.md")"
echo "ok: a finished bench comments its summary line, commit, target and run on the open issue, and in the run summary"

REF=docs/1-a-branch run "success 40/50 (80 %); duration p50 15.0 s, p95 17.0 s; causes: object_lost: 10" 12
grep -qF '`docs/1-a-branch` at `0123456`' "$GH_LOG.body" || fail "a dispatch on a branch named as main: $(cat "$GH_LOG.body")"
grep -qF "Target ≥ 90 %: not met." "$GH_LOG.body" || fail "$(cat "$GH_LOG.body")"
echo "ok: under the target, it says so"

run "evaluate: no /end_effector_pose: the brain does not see /joint_states" 12
grep -q '^issue comment 12 ' "$GH_LOG" && grep -qF "did not finish" "$GH_LOG.body" && grep -qF "$RUN_URL" "$GH_LOG.body" ||
  fail "$(cat "$GH_LOG.body")"
echo "ok: a bench that stopped before its summary line is reported too, with the run"

run "$FULL"
grep -q '^issue create --title Nightly bench results --label report ' "$GH_LOG" && grep -q '^issue comment 42 ' "$GH_LOG" ||
  fail "$(cat "$GH_LOG")"
echo "ok: no open issue: it is created once, then commented"

: > "$GH_LOG"
printf '%s\n' "$FULL" > "$tmp/bench.log"
(unset GITHUB_STEP_SUMMARY; GH_OPEN=12 bash "$report" "$tmp/bench.log" > /dev/null) || fail "without a step summary"
grep -q '^issue comment 12 ' "$GH_LOG" || fail "$(cat "$GH_LOG")"
echo "ok: outside Actions (no step summary file), the issue is still commented"
