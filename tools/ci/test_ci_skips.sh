#!/usr/bin/env bash
# Test of the two CI skips of #146, without network or GitHub: docs_only.sh (a PR that only touches documentation
# skips the build, the tests and the E2E job) and evaluation_due.sh (the nightly bench runs only when main changed),
# with a fake `gh` on PATH. pre-commit runs it when one of the scripts changes.
# Usage, from the repository root: bash tools/ci/test_ci_skips.sh
set -euo pipefail
dir=$(pwd)/tools/ci
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*"; exit 1; }

docs() { printf '%s\n' "$@" | bash "$dir/docs_only.sh" > /dev/null; }
docs README.md CHANGELOG.md docs/project/14_quality_pipeline_ci.md docs/project/adr/ADR-039-start-from-course-stubs.md \
  .github/PULL_REQUEST_TEMPLATE.md .github/ISSUE_TEMPLATE/task.yml docs/J3_visio_perception.pdf docs/project/img/a.png \
  || fail "documentation taken for code"
echo "ok: markdown, the docs folder's PDFs and images, the issue forms: documentation"
for code in ros2_ws/src/so101_driver/so101_driver/driver_node.py ros2_ws/src/so101_bringup/launch/sim.launch.py \
  ros2_ws/src/so101_bringup/config/driver.yaml ros2_ws/src/so101_perception/test/data/cube_00.png \
  docs/project/experiences/exp_ik_self_collision.py .github/workflows/ci.yml docker/Dockerfile.control \
  docker/control_entrypoint.sh tools/ci/guards.sh pyproject.toml .pre-commit-config.yaml .lycheeignore; do
  ! docs README.md "$code" || fail "$code taken for documentation"
done
echo "ok: code, tests and their data, experiments, workflows, Docker, launch, config, tooling: never documentation"
! bash "$dir/docs_only.sh" < /dev/null > /dev/null || fail "an empty change taken for documentation"
! docs "" || fail "a blank path taken for documentation"
echo "ok: no path at all is not documentation (a failed git diff runs everything)"

# Fake gh: `gh api` of the runs prints $RUNS ("id head_sha" lines, newest first), of a run's jobs the
# conclusion of its bench from $JOBS ("id conclusion" lines). GH_FAIL=1 makes every call fail.
mkdir "$tmp/bin"
cat > "$tmp/bin/gh" <<'EOF'
#!/usr/bin/env bash
[[ -z "${GH_FAIL:-}" ]] || exit 1
case $2 in
  */actions/workflows/nightly.yml/runs*) printf '%s\n' "$RUNS" ;;
  */actions/runs/*/jobs*) id=${2#*/actions/runs/}; id=${id%%/*}; printf '%s\n' "$JOBS" | awk -v id="$id" '$1 == id {print $2}' ;;
  *) exit 1 ;;
esac
EOF
chmod +x "$tmp/bin/gh"
export PATH="$tmp/bin:$PATH" GH_REPO=team/robotics
due() {  # due <expected true|false> <sha of main> <runs> <jobs> [GH_FAIL]
  local expected=$1 sha=$2 runs=$3 jobs=$4 gh_fail=${5:-}
  out=$(RUNS=$runs JOBS=$jobs GH_FAIL=$gh_fail bash "$dir/evaluation_due.sh" "$sha" 2> "$tmp/why") || fail "exit $?: $(cat "$tmp/why")"
  [[ "$out" = "evaluate=$expected" ]] || fail "expected evaluate=$expected, got '$out' ($(cat "$tmp/why"))"
}
due false aaa "9 aaa" "9 success"
echo "ok: the last bench passed on this commit of main: skipped"
due false aaa "9 aaa" "9 failure"
echo "ok: the last bench failed on this commit: it ran and reported, skipped (workflow_dispatch reruns it)"
due true bbb "9 aaa" "9 success"
echo "ok: main moved since the last bench: it runs"
due false aaa $'9 aaa\n8 aaa' $'9 skipped\n8 success'
due true bbb $'9 bbb\n8 aaa' $'9 skipped\n8 success'
echo "ok: skipped nights are passed over, back to the last bench that ran"
due true aaa $'9 aaa\n8 zzz' $'9 cancelled\n8 success'
echo "ok: a cancelled or timed-out bench gave no verdict: passed over too"
due true aaa "" ""
due true aaa "9 aaa" "9 success" 1
echo "ok: no bench in the listed runs, or gh failing: it runs"

# heavy.sh (#209): the PR's draft state when the run executes, not the event's. A push while draft and a "ready"
# in the same second start two runs, and the concurrency group cancels one; the survivor reads the live state.
heavy() { local draft=$1; shift; printf '%s\n' "$@" | bash "$dir/heavy.sh" "$draft" 2> /dev/null; }
[[ "$(heavy true ros2_ws/src/so101_driver/so101_driver/driver_node.py)" = heavy=false ]] || fail "a draft ran the heavy jobs"
[[ "$(heavy false ros2_ws/src/so101_driver/so101_driver/driver_node.py)" = heavy=true ]] || fail "a ready PR with code skipped them"
[[ "$(heavy false README.md docs/project/14_quality_pipeline_ci.md)" = heavy=false ]] || fail "a ready docs-only PR ran them"
[[ "$(heavy false tools/so101 README.md)" = heavy=true ]] || fail "a ready PR with tooling skipped them"
[[ "$(heavy "" tools/so101)" = heavy=true ]] || fail "an unknown draft state (failed lookup) skipped them"
[[ "$(heavy null tools/so101)" = heavy=true ]] || fail "a null draft state skipped them"
echo "ok: heavy.sh: a draft runs lint and guards only; ready (or unknown) runs everything unless documentation only"
