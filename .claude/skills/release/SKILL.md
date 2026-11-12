---
name: release
description: Cut a version of this project, from the nightly on the exact commit to the tag checked on the school copy. Use when a version's milestone is done (v2.x phase 2), when the nightly report says "releasable", or when asked to release or tag.
---

# Release a version

The rules are in `docs/project/15` §6 and ADR-028 (and its Update: a minor version per usable increment, one milestone per version). The mirror is described in `docs/project/16`. `tools/release.sh` does the checks and the writes; this skill orders the human steps around it. Merge nothing to `main` between step 2 and step 4. Always pass `-R FoxaxeWasTaken/Robotics` to `gh`.

1. **The milestone** `vX.Y.Z — …` has no open ticket but its "Release vX.Y.Z" ticket, the epics and those the last PR closes. Otherwise finish the tickets or move them to the next version. Read the quality gate of `main` now: a failed gate at the tag costs a PR and a nightly (#227).
2. **`CHANGELOG.md`**: `[Unreleased]` becomes `[X.Y.Z] - <date>`, with a short paragraph on the milestone, in the milestone's last PR (`open-pr` skill). Its own PR only if that one is already merged.
3. **The nightly on that commit**: once that PR is merged, dispatch the nightly (`gh workflow run nightly.yml -R FoxaxeWasTaken/Robotics --ref main`) and wait for its bench report on "Nightly bench results". The report's "Release:" line should say "releasable".
4. **`tools/release.sh --dry-run vX.Y.Z`**, then `tools/release.sh vX.Y.Z`, with `SONAR_TOKEN` set.
   - It refuses, with nothing written: a milestone with open tickets, no bench report on `origin/main`'s exact commit or a missed target, a failed quality gate, no CHANGELOG section.
   - Otherwise it does the rest: the tag, the GitHub Release (the CHANGELOG section and the bench run), the milestone closed and the next minor's opened, the mirror dispatched.
5. **After the script**:
   - the videos as Release assets (`gh release upload vX.Y.Z <file>`), never in Git;
   - the tag on the school copy (`git ls-remote --tags <school url>`, `docs/project/16`), never pushed there by hand;
   - a "Release" ticket for the new milestone (`new-ticket` skill);
   - the release ticket closed, with the links to the Release and to the nightly run.
