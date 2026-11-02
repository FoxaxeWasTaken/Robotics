---
name: open-pr
description: Prepare and open a pull request for this project, from the local checks to the "CI is green" box. Use when a ticket's work is committed and ready for review, or when asked to open, group or update a PR.
---

# Open a PR

The rules are in `CONTRIBUTING.md` (§2 workflow, §2.1 naming, §5 tests, §6 docs, §8 Definition of Done) and the template is `.github/PULL_REQUEST_TEMPLATE.md`. This skill only orders the steps. Always pass `-R FoxaxeWasTaken/Robotics` to `gh`.

## Before opening

1. **One commit per ticket**, conventional (`<type>(<scope>): …`, CONTRIBUTING §2.1). A group of tickets goes in one PR only when they touch the same files or the same roadmap step.
2. **Checks**, from the repository root: `pre-commit run --all-files`, `bash tools/ci/guards.sh`. Run the tests of each package you touched: `tools/so101 test <package>`, or the isolated run of the `isolated-stack` skill when a developer's stack is up.
3. **Docs the change makes wrong** (CONTRIBUTING §6): `git grep` each parameter, topic, file and ticket number you touched in `README.md`, `docs/project/` and `results/`. Fix the status tables, the counts and the "not handled" lines, in the present tense. Add a `CHANGELOG.md` entry under `[Unreleased]`.
4. **Branch up to date**: merge `origin/main` into it. Never rebase, never force-push.

## Opening

1. **Body** from the template:
   - one `Closes #<n>` line per ticket;
   - the decisions taken where a ticket or a doc was ambiguous;
   - the evidence (CONTRIBUTING §8): the commands and their output, the measurement, the RED → GREEN of the tests.

   Never put a local path (`/home/…`, `~/…`) in it: describe how to reproduce instead.
2. **Ready for review or draft**: CONTRIBUTING §2 step 6.
3. `gh pr create … --assignee @me`, then request the other members (CONTRIBUTING §2 step 6) one at a time with `gh pr edit --add-reviewer <login>`: a combined request can fail.
4. Move the tickets to **In review** on the board: `gh project item-edit 4 --owner FoxaxeWasTaken --url <issue url> --field Status --value "In review"`.

## After

- **Before editing the body**, re-read the live one (`gh pr view <n> --json body`): reviewers edit it on GitHub.
- When the CI is green, tick "CI is green" and give the run id. A draft runs only lint and guards, so tick it once the PR is ready and its full CI passed.
- A failed E2E job gets one rerun at most, then a `bug` issue (CONTRIBUTING §5).
