# Contributing

This guide is **the team's shared rulebook**. Details and rationale live in `docs/project/` (14: quality pipeline, 15: repository hygiene, 16: mirroring).

## 1. Setup (once)

```bash
git clone git@github.com:FoxaxeWasTaken/Robotics.git && cd Robotics
git remote add upstream git@github.com:JasonChmn/content-lerobot-bras-students.git
git remote set-url --push upstream DISABLED          # never push to the course repo
cp docker/.env.example docker/.env                   # ROS_DOMAIN_ID=69, the team's value
sudo apt install -y pipx && pipx install pre-commit  # plain pip is refused on Ubuntu 24.04 (PEP 668)
pre-commit install --install-hooks -t pre-commit -t commit-msg
cd docker && docker compose build
```

Full environment (WSL2, usbipd, RViz): `docs/project/02`.

## 2. Workflow

1. On the **GitHub Project** board, take a ticket from **Ready** (highest priority first, in your lane), assign it to yourself and move it to **In progress** (§8).
2. Create a branch named after the issue: `git switch -c feat/<issue#>-<topic>` (naming rules: §2.1).
3. Code **inside the containers** (`docker compose exec control bash`) — never run `colcon build` on the host.
4. Test locally:

   ```bash
   cd /ros2_ws && colcon build --symlink-install --packages-select <pkg>
   colcon test --packages-select <pkg> --event-handlers console_direct+ && colcon test-result --verbose
   bash tools/ci/guards.sh                              # from the repo root, on the host (needs the git repo)
   ```

5. Commit with a **conventional message**: `feat(driver): publish camera_info` (§2.1; the hook rejects anything else).
6. Open a PR with `Closes #<issue#>`, fill in the checklist; CODEOWNERS requests the review and the ticket moves to **In review**. A **draft** PR runs only lint, the guards and the PR title check: mark it **Ready for review** to run the build, the tests and the E2E job. A PR that only touches documentation never runs them (`docs/project/14` §5.4).
7. **Squash merge** once CI is green and the PR is approved. GitHub requires the branch to be **up to date with `main`**: if `main` moved, click *Update branch* (a merge; never rebase or force-push) and wait for the CI again. `main` gets no CI of its own (`docs/project/14` §5.4). A draft that waited for another PR: merge `main` into its branch (never rebase or force-push), then mark it ready, so that its full CI runs on the merged tree. The squash commit takes the **PR title**, so it must be conventional too (§2.1), CI checks it: the ticket closes and moves to **Done**, the branch is deleted.

### 2.1 Naming: branches, commits, PR titles

| What | Format | Example | Checked by |
| --- | --- | --- | --- |
| Branch | `<type>/<issue#>-<short-topic>` (lowercase, hyphens) | `feat/12-driver-camera-info` | review |
| Commit | `<type>(<scope>): <description>` | `fix(driver): reject NaN commands` | `commit-msg` hook (local) |
| PR title | same format: it becomes the squash commit on `main` | `feat(brain): add min-jerk trajectories` | CI `PR title (conventional commit)` |

- **Types**: `feat` new behavior · `fix` bug fix · `docs` · `test` · `refactor` no behavior change · `perf` · `build` Docker, dependencies, packaging · `ci` workflows, hooks · `chore` housekeeping · `revert`.
- **Scope** (optional, lowercase): the package or area: `driver`, `brain`, `perception`, `common`, `bringup`, `interfaces`, `connect4`, `docker`, `docs`, `ci`.
- **Description**: imperative ("add", not "added"), lowercase start, no final period, at most 72 characters.
- **Breaking change** of the interface contract (`docs/project/01` §3): `!` after the type or scope (`feat(interfaces)!: rename /object_position`), explained in the PR body.
- **Body** (optional): why, not how; `Co-authored-by:` trailer when pairing (C-07).

The hook and the CI check the type, the lowercase scope and the `: description` part; the imperative form and the 72 characters are checked in review.

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
- No magic numbers: constants in `so101_common`, tunables as ROS parameters (YAML in `so101_bringup/config/`).
- **Do not modify** `core/`, `sim/`, `ros2_ws/src/so101_description/` (course kit). Provided files you have not rewritten yet are excluded from lint so they stay identical to upstream; remove a file from the exclusion list in `pyproject.toml` when you rewrite it.

## 5. Tests

| Type | Where | When to write one |
| --- | --- | --- |
| Unit (pytest) | `<pkg>/test/test_*.py` | Any pure logic (conversions, IK, FSM, geometry, detection) |
| Component (launch_testing) | `<pkg>/test/test_*_launch.py` | Rates, commands, services of one node |
| E2E | `ros2 run so101_bringup evaluate` | Full pick & place (N episodes) |

A flaky test never lands in `main`: quarantine marker + issue.

## 6. Decisions and documentation

- A structural choice gets an **ADR** (`docs/project/adr/`, template in `adr/README.md`). Accepted ADRs are not rewritten: a new one supersedes them.
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
- **Fields**: Lane (A, B, C, Shared, `docs/project/09` §1.2), Priority (P0 critical path · P1 needed for the final defense · P2 optional), Size (XS < 2 h · S ≤ ½ day · M ≤ 1 day · L ≤ 2–3 days). Dependencies use GitHub's native **blocked by** relationship, not labels.
- **Labels**: `task`, `bug`, `epic`, `safety` (moves the real arm: [SAFE] review, sim first, `docs/project/07` §7), `staff-question`, and one or more `area:*`.

**Anatomy of a ticket** (the issue template, `.github/ISSUE_TEMPLATE.md`): Roadmap step · Goal · Scope · Out of scope · **Done when** · Files and interfaces · References · Review focus. Write it so that **a teammate or an AI agent can do it without asking**: exact paths, checkable criteria (a command, a test, a threshold), what not to touch.

**Definition of Ready** (Backlog → Ready): every *blocked by* ticket is Done; *Done when* is checkable; files and interface impact are named; open questions answered or listed in the ticket.

**Definition of Done** (applies to every ticket, on top of its own *Done when*):

- [ ] Merged through a reviewed PR with green CI (lint, guards, build, tests).
- [ ] Rules of §3 respected (only `/joint_command`, no ground truth, no provided extrinsics, kit untouched).
- [ ] A test covers the new behavior; measurements reported in the PR (and in `results/` for milestones).
- [ ] Docs, parameters tables and ADRs updated in the same PR when they changed; the reviewer can explain the change.

**Working with an AI agent**: give it the issue; it reads `AGENTS.md` (commands, prohibitions, traps; `CLAUDE.md` imports it) and this file. The ticket body is the specification; *Out of scope* is binding; if a referenced document is ambiguous, the agent decides, says so in the PR and updates that document in the same PR. The author stays responsible for the PR (C-07).
