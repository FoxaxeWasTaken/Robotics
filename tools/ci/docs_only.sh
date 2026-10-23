#!/usr/bin/env bash
# Exit 0 when every path read on stdin (one per line) is documentation: the PR skips the build, the tests and
# the E2E job (#146). Anything else exits 1: code, tests and their data, workflows, Docker, launch and config
# files always get the full CI, and so does an empty list (a failed git diff).
# Usage: git diff --name-only --no-renames <base> <head> | bash tools/ci/docs_only.sh
n=0
while IFS= read -r path; do
  n=$((n + 1))
  case $path in
    *.md | docs/*.pdf | docs/*.png | docs/*.jpg | docs/*.svg | .github/ISSUE_TEMPLATE/*) ;;
    *) echo "Not documentation: '$path'"; exit 1 ;;
  esac
done
[ "$n" -gt 0 ] && echo "Documentation only ($n files)."
