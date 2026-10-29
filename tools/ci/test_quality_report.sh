#!/usr/bin/env bash
# Test of quality_report.sh (#179), without network or GitHub: a fake `curl` answers the SonarCloud web API from
# canned JSON, a fake `gh` records the issue calls. pre-commit runs it when either script or post_report.sh changes.
# Usage, from the repository root: bash tools/ci/test_quality_report.sh
set -euo pipefail
report=$(pwd)/tools/ci/quality_report.sh
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*"; exit 1; }

# Fake curl: it logs the URL; with $CURL_FAIL it fails as curl -f does on an HTTP error; else it prints
# $tmp/api/<task|gate|measures>.json, and the ce task is PENDING for the first $PENDING calls.
mkdir "$tmp/bin" "$tmp/api"
cat > "$tmp/bin/curl" <<'EOF'
#!/usr/bin/env bash
url=${*: -1}
echo "$url" >> "$CURL_LOG"
[[ -z ${CURL_FAIL:-} ]] || { echo "curl: (22) The requested URL returned error: 401" >&2; exit 22; }
case $url in
  */api/ce/task*)
    if (( $(grep -c /api/ce/task "$CURL_LOG") <= ${PENDING:-0} )); then echo '{"task":{"status":"PENDING"}}'
    else cat "$API/task.json"; fi ;;
  */api/qualitygates/project_status*) cat "$API/gate.json" ;;
  */api/measures/component*) cat "$API/measures.json" ;;
  *) exit 22 ;;
esac
EOF
# Fake gh, as in test_bench_report.sh: `issue list` prints $GH_OPEN, --body-file copied to $GH_LOG.body.
cat > "$tmp/bin/gh" <<'EOF'
#!/usr/bin/env bash
echo "$*" >> "$GH_LOG"
[[ "$1 $2" == "issue list" ]] && printf '%s\n' "${GH_OPEN:-}"
[[ "$1 $2" == "issue create" ]] && echo "https://github.com/team/robotics/issues/42"
prev=""; for a in "$@"; do [[ $prev == --body-file ]] && cp "$a" "$GH_LOG.body"; prev=$a; done
exit 0
EOF
chmod +x "$tmp/bin/curl" "$tmp/bin/gh"
export PATH="$tmp/bin:$PATH" API="$tmp/api" CURL_LOG="$tmp/curl.log" GH_LOG="$tmp/gh.log" GH_REPO=team/robotics \
  SONAR_TOKEN=t0ken SONAR_POLL_S=0 REF=main SHA=0123456789abcdef RUN_URL=https://github.com/team/robotics/actions/runs/7 \
  ARTIFACT=trends-3

echo 'ceTaskId=AZ-task-1' > "$tmp/report-task.txt"
echo '{"task":{"status":"SUCCESS","analysisId":"AZ-analysis-1"}}' > "$tmp/api/task.json"
PASSED='{"projectStatus":{"status":"OK","conditions":[{"status":"OK","metricKey":"new_coverage","comparator":"LT","errorThreshold":"80","actualValue":"91.0"}]}}'
FAILED='{"projectStatus":{"status":"ERROR","conditions":[{"status":"ERROR","metricKey":"new_coverage","comparator":"LT","errorThreshold":"80","actualValue":"29.3"},{"status":"OK","metricKey":"new_duplicated_lines_density","comparator":"GT","errorThreshold":"3","actualValue":"0.0"}]}}'
echo '{"component":{"measures":[{"metric":"coverage","value":"82.6"},{"metric":"new_coverage","period":{"value":"91.0"}},{"metric":"violations","value":"12"},{"metric":"new_violations","period":{"value":"3"}}]}}' \
  > "$tmp/api/measures.json"
printf '%s\n' "date,sha,metric,p50_ms,worst_ms" "2026-10-29,0123456,test_ik_pose,12.345,20.100" \
  "2026-10-29,0123456,control_period,20.000,24.500" > "$tmp/trends.csv"

run() {  # run [trends.csv]: the report, with a step summary file and the open issue #12
  : > "$CURL_LOG" && : > "$GH_LOG" && rm -f "$GH_LOG.body" && : > "$tmp/summary.md"
  GITHUB_STEP_SUMMARY="$tmp/summary.md" GH_OPEN=12 bash "$report" "${1:-$tmp/trends.csv}" "$tmp/report-task.txt" \
    > "$tmp/out.log" 2>&1 || fail "exit $?: $(cat "$tmp/out.log")"
}
has() { for e in "$@"; do grep -qF -- "$e" "$GH_LOG.body" || fail "no '$e' in the comment: $(cat "$GH_LOG.body")"; done; }

echo "$PASSED" > "$tmp/api/gate.json"
run
grep -q '^issue comment 12 --body-file' "$GH_LOG" || fail "$(cat "$GH_LOG")"
has "\`main\` at \`0123456\`" "quality gate passed" "Coverage 82.6 % (new code 91.0 %)" "3 new issues, 12 open" \
  "https://sonarcloud.io/project/overview?id=FoxaxeWasTaken_Robotics" "| test_ik_pose | 12.345 | 20.100 |" \
  "| control_period | 20.000 | 24.500 |" "$RUN_URL" "trends-3"
grep -q 'analysisId=AZ-analysis-1' "$CURL_LOG" || fail "the gate of another analysis: $(cat "$CURL_LOG")"
cmp -s "$GH_LOG.body" "$tmp/summary.md" || fail "the run summary differs from the comment: $(cat "$tmp/summary.md")"
echo "ok: the gate of this run's analysis, the coverage, the issues and the trends, on the issue and in the run summary"

echo "$FAILED" > "$tmp/api/gate.json"
run
has "quality gate failed" "\`new_coverage\` 29.3, required ≥ 80"
! grep -qF new_duplicated_lines_density "$GH_LOG.body" || fail "a passed condition listed: $(cat "$GH_LOG.body")"
echo "ok: a failed gate lists its failed conditions only"

PENDING=2 run
[[ $(grep -c /api/ce/task "$CURL_LOG") == 3 ]] && has "quality gate failed" || fail "$(cat "$CURL_LOG")"
echo "ok: it waits for SonarCloud to process the analysis"

CURL_FAIL=1 run
has "SonarQube Cloud: no result" "| test_ik_pose | 12.345 | 20.100 |" "$RUN_URL"
! grep -qF t0ken "$GH_LOG.body" || fail "the token in the comment"
echo "ok: a failed SonarCloud call still posts the trends"

run "$tmp/missing.csv"
has "No trends.csv" "quality gate failed"
echo "ok: without trends.csv, the analysis is still posted"
