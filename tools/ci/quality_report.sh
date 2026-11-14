#!/usr/bin/env bash
# Nightly quality report (#179): the SonarQube Cloud analysis of this run (quality gate, coverage, issues) and the rows
# of trends.csv (#60), posted by post_report.sh (run summary, one comment on the issue "Nightly bench results"). A failed
# SonarCloud call, or no trends.csv, is said and the rest still posted. Needs `gh` allowed to write issues (GH_TOKEN),
# SONAR_TOKEN, REF, SHA, RUN_URL, ARTIFACT in the environment, and the scanner's report-task.txt (its ceTaskId).
# Usage, from the repository root: bash tools/ci/quality_report.sh <trends.csv> <report-task.txt>
# (tested by test_quality_report.sh)
set -euo pipefail
host=https://sonarcloud.io
project=$(sed -n 's/^sonar\.projectKey=//p' "$(dirname "${BASH_SOURCE[0]}")/../../sonar-project.properties")

api() {  # api <path?query>: SonarCloud web API, JSON on stdout; fails on an HTTP error
  # the token through a config on stdin: not in the command line (ps) nor in the log
  local path=$1
  curl -fsS --max-time 30 -K - "$host/api/$path" <<<"header = \"Authorization: Bearer $SONAR_TOKEN\""
}

sonar() {  # sonar <report-task.txt>: the analysis as Markdown; errors on stderr, non-zero status
  local task json status analysis
  task=$(sed -n 's/^ceTaskId=//p' "$1" 2> /dev/null)
  [[ -n $task ]] || { echo "no ceTaskId in $1" >&2; return 1; }
  for _ in {1..60}; do  # the analysis is processed after the scan, usually within a minute
    json=$(api "ce/task?id=$task") || return 1
    read -r status analysis <<< "$(python3 -c '
import json, sys
task = json.loads(sys.argv[1])["task"]
print(task["status"], task.get("analysisId", ""))' "$json")"
    [[ $status == PENDING || $status == IN_PROGRESS ]] || break
    sleep "${SONAR_POLL_S:-5}"
  done
  [[ $status == SUCCESS ]] || { echo "analysis $task: $status" >&2; return 1; }
  json=$(api "qualitygates/project_status?analysisId=$analysis") || return 1
  python3 -c '
import json, sys
gate = json.loads(sys.argv[1])["projectStatus"]
failed = [c for c in gate["conditions"] if c["status"] == "ERROR"]
if not failed:
    print("**SonarQube Cloud: quality gate passed.**")
else:
    print("**SonarQube Cloud: quality gate failed** on:\n")
    for c in failed:  # LT: an error under the threshold, GT: over it
        need = "≥" if c["comparator"] == "LT" else "≤"
        print("- `{}` {}, required {} {}".format(c["metricKey"], c["actualValue"], need, c["errorThreshold"]))
' "$json" || return 1
  echo
  json=$(api "measures/component?component=$project&branch=$REF&metricKeys=coverage,new_coverage,violations,new_violations") ||
    return 1
  python3 -c '
import json, sys
measures = json.loads(sys.argv[1])["component"]["measures"]
# A new-code measure has no "value": SonarQube Cloud gives "periods", a list of one
m = {x["metric"]: x.get("value", (x.get("periods") or [{}])[0].get("value", "?")) for x in measures}
if m.get("new_coverage", "?") != "?":
    m["new_coverage"] = "{:.1f}".format(float(m["new_coverage"]))  # 93.40498290180753
print("Coverage {} % (new code {} %), {} new issues, {} open.".format(
    m.get("coverage", "?"), m.get("new_coverage", "?"), m.get("new_violations", "?"), m.get("violations", "?")))
' "$json"
}

body=$(mktemp)
{
  echo "### Nightly quality, $(date -u +%F)"
  echo
  echo "\`$REF\` at \`${SHA:0:7}\`: every package's tests with coverage, the IK benchmarks, the control period."
  echo
  if analysis=$(sonar "$2" 2> "$body.err"); then
    echo "$analysis"
  else
    echo "**SonarQube Cloud: no result** ($(tail -n 1 "$body.err")). See the run's log."
  fi
  echo
  echo "Dashboard: $host/project/overview?id=$project"
  echo
  if [[ -f $1 ]]; then
    echo "| Trend (\`08\` §2) | p50 (ms) | worst (ms) |"
    echo "| --- | --- | --- |"
    awk -F, 'NR > 1 { printf "| %s | %s | %s |\n", $3, $4, $5 }' "$1"
  else
    echo "**No trends.csv**: the tests or the benchmarks stopped before it. See the run's log."
  fi
  echo
  echo "Run: $RUN_URL (trends.csv: artifact \`$ARTIFACT\`, kept 90 days)."
} > "$body"
cat "$body.err" >&2
bash "$(dirname "${BASH_SOURCE[0]}")/post_report.sh" "$body"
