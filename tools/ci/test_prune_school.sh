#!/usr/bin/env bash
# Test of prune_school.sh (#216) without network: a throwaway bare repository stands for the school copy, with
# main, a tag and branches that must go (Dependabot's, a member's, names that start like main).
# pre-commit runs it when either script changes. Usage, from the repository root: bash tools/ci/test_prune_school.sh
set -euo pipefail
src=$(pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*"; exit 1; }
git init -q --bare "$tmp/school.git"
git init -q "$tmp/work" && cd "$tmp/work"
git config user.email t@t && git config user.name t
git commit -q --allow-empty -m init && git tag v1.0.0
git remote add school "$tmp/school.git"
git push -q school HEAD:refs/heads/main --tags
for b in dependabot/github_actions/actions/setup-python-7.1.0 feature main-old mainx; do git push -q school "HEAD:refs/heads/$b"; done
bash "$src/tools/ci/prune_school.sh" school > "$tmp/out.log" || fail "exit $?: $(cat "$tmp/out.log")"
[[ "$(git ls-remote --heads school | cut -f2)" = refs/heads/main ]] || fail "left: $(git ls-remote --heads school | cut -f2 | tr '\n' ' ')"
[[ "$(git ls-remote --tags school | cut -f2)" = refs/tags/v1.0.0 ]] || fail "the tag went"
grep -qF "dependabot/github_actions/actions/setup-python-7.1.0" "$tmp/out.log" || fail "the deleted branches are not listed: $(cat "$tmp/out.log")"
bash "$src/tools/ci/prune_school.sh" school > "$tmp/out.log" || fail "a second run, nothing to delete: exit $?"
echo "ok: prune_school.sh deletes every branch of the school copy but main (main-old and mainx too), keeps the tags, and is a no-op when clean"
