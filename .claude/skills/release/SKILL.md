---
name: release
description: Cut a version of this project, from the nightly on the exact commit to the tag checked on the school copy. Use when a functional milestone is reached (v1.1.0 real arm, v2.x phase 2) or when asked to release or tag.
---

# Release a version

The versioning rules are in `docs/project/15` §6, ADR-020 and ADR-028. The mirror is described in `docs/project/16`. This skill only orders the steps. Merge nothing to `main` between step 1 and step 6. Always pass `-R FoxaxeWasTaken/Robotics` to `gh`.

0. **The milestone**: `vX.Y.Z — …` has no open ticket but its "Release vX.Y.Z" ticket (ADR-028 Update). Otherwise finish or move them first.
1. **The commit to tag**: the head of `main` (`git rev-parse origin/main`). Its scheduled nightly must have run on that exact SHA. If `main` did not change since the last nightly, dispatch it: `gh workflow run nightly.yml -R FoxaxeWasTaken/Robotics --ref main`.
2. **Its evidence**: check the nightly's bench report on "Nightly bench results" (target in `docs/project/08` §2). Then check the SonarQube Cloud quality gate of `main`, from the nightly's quality report or SonarQube.
3. **`CHANGELOG.md`**: `[Unreleased]` becomes `[x.y.z] - <date>`, with a short paragraph on the milestone. This goes through its own PR (`open-pr` skill).
4. **Tag and Release**, on the commit of step 1 once that PR is merged:
   - `git tag -a vX.Y.Z <sha> -m …`, then `git push origin vX.Y.Z`;
   - `gh release create vX.Y.Z -R FoxaxeWasTaken/Robotics`, with the CHANGELOG section as notes, the bench report and the links to the videos;
   - the videos themselves as Release assets, never in Git.
5. **The school copy**: the mirror runs on pushes to `main`, not on tags. Run it after the tag with `gh workflow run mirror.yml -R FoxaxeWasTaken/Robotics --ref main`. Then check that the tag is there with `git ls-remote --tags <school url>` (`docs/project/16`). Never push to the school repository yourself: only the mirror does.
6. **Close** the milestone, and open the next minor's if it is missing (`gh api -X POST repos/FoxaxeWasTaken/Robotics/milestones -f title=…`) with its "Release" ticket (`new-ticket` skill). Close the release ticket with the links to the Release and to the nightly run.
