# Contributing

This guide is **the team's shared rulebook**. Details and rationale live in `docs/project/` (14: quality pipeline, 15: repository hygiene, 16: mirroring).

## 1. Setup (once)

```bash
git clone git@github.com:FoxaxeWasTaken/Robotics.git && cd Robotics
git remote add upstream git@github.com:JasonChmn/content-lerobot-bras-students.git
git remote set-url --push upstream DISABLED          # never push to the course repo
cp docker/.env.example docker/.env                   # ROS_DOMAIN_ID=69, the team's value
pipx install pre-commit                              # or your distribution's package; plain pip is refused where PEP 668 applies
pre-commit install --install-hooks -t pre-commit -t commit-msg
tools/so101 build                                    # the images, ~10 min the first time
```

Prerequisites (a Linux host with Docker and an X server; Windows through WSL2) and what the team tested on: the [README](README.md#prerequisites). Full environment (display, USB devices, the WSL2 specifics): `docs/project/02`. Everyday commands: `tools/so101 --help` (start the stack, a cycle, the bench, the tests; `docs/project/13` §1).

## 2. Workflow

1. On the **GitHub Project** board, take a ticket from **Ready** (highest priority first, in your lane), assign it to yourself and move it to **In progress** (§8).
2. Create a branch named after the issue: `git switch -c feat/<issue#>-<topic>` (naming rules: §2.1).
3. Start the stack with `tools/so101 up` and code; build and run ROS **inside the containers** (`tools/so101 shell`), never `colcon build` on the host.
4. **Write the test first** (§5), then the code. Test locally:

   ```bash
   tools/so101 test <pkg> -k <name>                      # while coding: the tests you work on (options go to pytest)
   tools/so101 test <pkg>                                # then the whole package; tools/so101 test: all of ours,
                                                         # the stack stopped (the planner's timing tests need the CPU)
   pre-commit run --all-files && bash tools/ci/guards.sh # host, repo root: lint, format, the project rules
   tools/so101 bench --perception -n 10                  # a change that can affect the cycle: the bench
   ```

5. Commit with a **conventional message**: `feat(driver): publish camera_info` (§2.1; the hook rejects anything else). One commit per ticket.
6. Open the PR with one `Closes #<issue#>` line per ticket, assign yourself and **request the review of the other members**: `.github/CODEOWNERS` is indicative, any member may approve (`docs/project/14` §10). Fill in the checklist and show the **evidence**: the commands run and their output, or the measurement (§8). The ticket moves to **In review**.
   - Open it **ready for review**. It stays a **draft** only when it is stacked on another open PR (its branch starts from that PR's branch).
   - A draft runs only lint, the guards and the PR title check; marking it ready runs the build, the tests and the E2E job. A PR that only touches documentation never runs them (`docs/project/14` §5.4). Before ticking "CI is green", check that the run's "Build, tests and E2E needed?" step printed `heavy=true` (unless documentation only): the run a draft made is not the ready PR's CI.
7. **Prepare the merge**: merge `main` into the branch (`git fetch origin && git merge origin/main`, or *Update branch* on GitHub; never rebase or force-push), wait for the CI to pass on the merged tree, then **squash merge** once the PR is approved. GitHub refuses the merge while the branch is behind `main`, and `main` gets no CI of its own (`docs/project/14` §5.4). A stacked draft: once the PR below it is merged, merge `main` into its branch, then mark it ready.
8. The squash commit takes the **PR title**, so it must be conventional too (§2.1), CI checks it. The ticket closes and moves to **Done**, the branch is deleted.

### 2.1 Naming: branches, commits, PR titles

| What | Format | Example | Checked by |
| --- | --- | --- | --- |
| Branch | `<type>/<issue#>-<short-topic>` (lowercase, hyphens) | `feat/12-driver-camera-info` | review |
| Commit | `<type>(<scope>): <description>` | `fix(driver): reject NaN commands` | `commit-msg` hook (local) |
| PR title | same format: it becomes the squash commit on `main` | `feat(brain): add min-jerk trajectories` | CI `PR title (conventional commit)` |

- **Types**: `feat` new behavior · `fix` bug fix · `docs` · `test` · `refactor` no behavior change · `perf` · `build` Docker, dependencies, packaging · `ci` workflows, hooks · `chore` housekeeping · `revert`.
- **Scope** (optional, lowercase): the package or area: `driver`, `brain`, `perception`, `common`, `bringup`, `interfaces`, `connect4`, `docker`, `docs`, `ci`, `tooling` (`tools/`).
- **Description**: imperative ("add", not "added"), lowercase start, no final period, at most 72 characters.
- **Breaking change** of the interface contract (`docs/project/01` §3): `!` after the type or scope (`feat(interfaces)!: rename /object_position`), explained in the PR body.
- **Body** (optional): why, not how; `Co-authored-by:` trailer when pairing (C-07).

The hook and the CI check the type, the lowercase scope and the `: description` part; the imperative form and the 72 characters are checked in review.

### 2.2 Reviewing

Any member may review and approve any PR. The author requests the reviews (step 6) and merges (step 7).

- **Start from the ticket's *Review focus***, then check:
  - the evidence in the PR: the commands and their output, the measurements;
  - the tests: RED → GREEN (§5);
  - the rules of §3;
  - the docs, updated with the code.
- **Comment on the lines**, and ask rather than guess. The author answers each thread, then resolves it or lets the reviewer do it.
- **The ruleset of `main`** (`docs/project/14` §10):
  - 1 approval, and every conversation resolved, before the merge;
  - the required checks: lint, the project-rule guards, the build and tests of both images, the E2E job (3 pick & place out of 3), the PR title;
  - the branch up to date with `main`, and squash merge only;
  - new commits dismiss the approvals: after a push (an *Update branch* included), check that the approval is still there, else ask again.

The recurring procedures (a ticket, a PR, a measurement or an ADR, a run next to someone's stack, a release, the nightly triage) also exist as Claude Code skills in `.claude/skills/`: each gives the order of the steps and points to this file or to `docs/project/`, which stay the reference.

## 3. Non-negotiable rules (from the project brief)

- Motor commands go **only** through `/joint_command`; brain and perception **never** import the backend (C-05).
- **No ground truth** (`get_object_position`, …) in the pipeline — only in `test/` and in the evaluation bench (C-06).
- The final perception does not use `get_camera_extrinsics()` (C-23).
- Everyone commits **their own** work; when pairing, add `Co-authored-by:` (C-07).
- **Never** push to the school repository: it is a mirror fed by `mirror.yml` (`docs/project/16`).
- **Real arm**: every new command is tested in sim first with the same parameters — checklist in `docs/project/07` §7.

The first three rules are checked in CI by `tools/ci/guards.sh`; the others rely on review and on the checklist of `docs/project/07` §7.

## 4. Style and quality

- Python: **ruff** (lint + format, config in `pyproject.toml`). Run `pre-commit run --all-files` before a large PR.
- Non-trivial logic lives **outside ROS callbacks**, in pure modules covered by unit tests.
- No magic numbers: constants in `so101_common`, tunables as ROS parameters declared with a description and a range. Every tunable is in its base YAML (`so101_bringup/config/`, `so101_perception/config/`); the real arm overrides only the keys of `docs/project/01` §3.3 (`*_real.yaml`).
- **Do not modify** `core/`, `sim/`, `ros2_ws/src/so101_description/` (course kit). Provided files you have not rewritten yet are excluded from lint so they stay identical to upstream; remove a file from the exclusion list in `pyproject.toml` when you rewrite it.

## 5. Tests

**Test first (TDD).** For a new behavior or a bug fix:

1. write the test that shows it;
2. run it and watch it fail, for the expected reason (not an import error or a typo);
3. write the code until it passes, then run the package's whole suite.

The PR shows both runs (RED → GREEN). A bug gets its regression test before its fix. A test that passes at once proves nothing: change it until it fails without the code.

| Type | Where | When to write one |
| --- | --- | --- |
| Unit (pytest) | `<pkg>/test/test_*.py` | Any pure logic (conversions, IK, FSM, geometry, detection); a rule that must hold for every input gets a property test (Hypothesis, `docs/project/08` §5) |
| Component (launch_testing) | `<pkg>/test/test_*_launch.py` | Rates, commands, services of one node |
| Sim-backed | `so101_bringup/test/` | Anything that needs `so101_sim` (closed loop, IK accuracy, launch files): never under `so101_brain` or `so101_perception` |
| E2E | `tools/so101 bench` (`ros2 run so101_bringup evaluate`) | Full pick & place, N episodes. CI runs 3 on every PR (`e2e-smoke`, a required check: 3 successes out of 3) and 50 in the nightly each night `main` changes |
| Scripts | `tools/ci/test_*.sh`, run by pre-commit | A CI or tooling script: its behavior without network or Docker (fake `gh`, `--dry-run`) |

The safety modules also have mutation testing (mutmut, run by hand, `docs/project/08` §1).

**A flaky test never stays in `main`.** Open a `bug` issue with the failing run, then fix the cause, or the test, in its own PR, as #141 and #142 did. Never rerun until it passes. The E2E job is the one exception, since it runs the whole stack: one rerun of the failed job at most, and a second failure is a `bug` issue.

## 6. Decisions and documentation

- **A PR updates every doc it makes wrong, in the same PR**, not only the doc of the file it changes. Before opening it, `git grep` what the change touches (the parameter, topic, file, state, ticket number) in `README.md`, `AGENTS.md`, `docs/project/` and `results/`, and fix:
  - the status tables: the README's Status and Measured results, `01` §2, `03` §0, `00` §7, the roadmap's checkboxes;
  - what the change settles elsewhere: "not handled", "to add", "planned" in a risk (`10`), a runbook row (`13`) or a report's verdict (`results/`, as a dated note);
  - the counts and the indexes: `docs/project/README.md`, `experiences/README.md`.
- **Docs say what exists, in the present** ("the driver clamps every command"), never the path to it ("since step 1.1", "until 6.1a"): the CHANGELOG and git keep the history. Dated records stay dated: the project history, the cross-review log, the `results/` reports.
- A structural choice gets an **ADR** (`docs/project/adr/`, template in `adr/README.md`). Accepted ADRs are not rewritten: add an *Update* section, or a new ADR that supersedes them.
- A new measurement gets a `MES-xx` row in `docs/project/11` §4, its script in `docs/project/experiences/`; update the counts in `docs/project/README.md`.
- Changing the **interface contract** (`docs/project/01` §3) requires both sides' owners to agree.
- Diagrams are **Mermaid**, and their rendering is checked in CI.

## 7. Syncing with the course kit

```bash
git fetch upstream && git switch -c chore/sync-upstream && git merge upstream/main
git rev-parse upstream/main > .github/upstream-sha    # the course commit now merged (#137)
# resolve conflicts, then open a "chore: sync upstream" PR
```

The nightly `upstream` job opens one issue, "Course repository: new commits", when the course's `main` moves past the commit recorded in `.github/upstream-sha`, and updates it on the next nights (#137). The sync PR writes the merged commit there and closes that issue with its `Closes #<n>` line.

## 8. Tickets and the GitHub Project

All planned work lives in **one GitHub Project** (board + backlog) linked to this repository. Setup, fields, views and automations: `docs/project/15` §12.

- **One ticket = a few hours to two days of work**. Bigger? Split it. A PR closes one ticket or a **group of related tickets** (same files or same roadmap step: fewer conflicts and re-reviews), with one commit and one `Closes #<n>` line per ticket. Tickets are grouped under one **epic per roadmap stage** (sub-issues; *phase 1 / phase 2* are the brief's parts, `docs/project/12`) and carry the roadmap step in their title: `[1.3a] Publish camera_info and use the optical frame`.
- **Statuses**: Backlog → **Ready** → In progress → In review → Done. At most **five tickets In progress per person**.
- **Milestone**: the version the ticket ships in (`v1.3.0`…; ADR-028 Update, `docs/project/15` §6), set when the ticket is created. A PR cannot merge while a ticket it closes has none.
- **Fields**: Lane (A, B, C, Shared, `docs/project/09` §1.2), Priority (P0 critical path · P1 needed for the final defense · P2 optional), Size (XS < 2 h · S ≤ ½ day · M ≤ 1 day · L ≤ 2–3 days). Dependencies use GitHub's native **blocked by** relationship, not labels.
- **Labels**: `task`, `bug`, `epic`, `safety` (moves the real arm: [SAFE] review, sim first, `docs/project/07` §7), `staff-question`, and one or more `area:*`. `report` marks an automatic report, not a work item (the pinned "Nightly bench results"): the project views leave it out.

**Anatomy of a ticket** (the issue template, `.github/ISSUE_TEMPLATE.md`): Roadmap step · Goal · Scope · Out of scope · **Done when** · Files and interfaces · References · Review focus. Write it so that **a teammate or an AI agent can do it without asking**: exact paths, checkable criteria (a command, a test, a threshold), what not to touch.

**Definition of Ready** (Backlog → Ready): it has a milestone; every *blocked by* ticket is Done; *Done when* is checkable; files and interface impact are named; open questions answered or listed in the ticket.

**Definition of Done** (applies to every ticket, on top of its own *Done when*):

- [ ] Merged through a reviewed PR with green CI (lint, guards, build, tests).
- [ ] Rules of §3 respected (only `/joint_command`, no ground truth, no provided extrinsics, kit untouched).
- [ ] A test covers the new behavior; measurements reported in the PR (and in `results/` for milestones).
- [ ] Docs, parameters tables and ADRs updated in the same PR when they changed; the reviewer can explain the change.

**Working with an AI agent**: give it the issue; it reads `AGENTS.md` (commands, prohibitions, traps; `CLAUDE.md` imports it) and this file. The ticket body is the specification; *Out of scope* is binding; if a referenced document is ambiguous, the agent decides, says so in the PR and updates that document in the same PR. The author stays responsible for the PR (C-07).
