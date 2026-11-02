#!/usr/bin/env bash
# The version milestones (ADR-028 Update, #210), for tools/release.sh and the nightly's bench report (#211):
#   milestone.sh next <owner/repo>           the oldest open version milestone: "<number>\t<title>\t<state>"
#   milestone.sh find <owner/repo> vX.Y.Z    that version's milestone, open or closed, same format; nothing if none
#   milestone.sh blockers <owner/repo> <n>   its open issues but its release ticket and the epics: one "#n" per line
# Needs gh (GH_TOKEN) and python3. Tested by test_release.sh.
set -euo pipefail
command=${1:-} repo=${2:-}
case $command in
  next | find)
    gh api "repos/$repo/milestones?state=all&per_page=100" | python3 -c '
import json, re, sys
command, wanted = sys.argv[1], sys.argv[2]
def version(title):
    m = re.match(r"v(\d+)\.(\d+)\.(\d+)( |$)", title)
    return tuple(int(g) for g in m.groups()[:3]) if m else None
milestones = [m for m in json.load(sys.stdin) if version(m["title"])]
if command == "next":
    milestones = sorted((m for m in milestones if m["state"] == "open"), key=lambda m: version(m["title"]))[:1]
else:
    milestones = [m for m in milestones if m["title"].split()[0] == wanted]
for m in milestones:
    print("%s\t%s\t%s" % (m["number"], m["title"], m["state"]))
' "$command" "${3:-}"
    ;;
  blockers)
    gh api "repos/$repo/issues?milestone=${3:?milestone number}&state=open&per_page=100" | python3 -c '
import json, sys
for issue in json.load(sys.stdin):
    labels = {label["name"] for label in issue["labels"]}
    if "pull_request" in issue or "epic" in labels or "Release v" in issue["title"]:
        continue
    print("#%s" % issue["number"])
'
    ;;
  *) echo "milestone.sh: next, find or blockers" >&2; exit 2 ;;
esac
