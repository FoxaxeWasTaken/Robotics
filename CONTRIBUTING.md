# Contributing

This guide is **the team's shared rulebook**. Details and rationale live in `docs/project/` (14: quality pipeline, 15: repository hygiene, 16: mirroring).

## 1. Setup (once)

```bash
git clone git@github.com:<team>/<repo>.git && cd <repo>
git remote add upstream git@github.com:JasonChmn/content-lerobot-bras-students.git
git remote set-url --push upstream DISABLED          # never push to the course repo
cp docker/.env.example docker/.env                   # team-specific ROS_DOMAIN_ID
pip install pre-commit && pre-commit install --install-hooks -t pre-commit -t commit-msg
cd docker && docker compose build
```

Full environment (WSL2, usbipd, RViz): `docs/project/02`.

## 2. Workflow

1. On the **GitHub Project** board, take a ticket from **Ready** (highest priority first, in your lane), assign it to yourself and move it to **In progress** (§8).
2. Create a branch named after the issue: `git switch -c feat/<issue#>-<topic>` (types: `feat`, `fix`, `docs`, `test`, `refactor`, `perf`, `build`, `ci`, `chore`).
3. Code **inside the containers** (`docker compose exec control bash`) — never run `colcon build` on the host.
4. Test locally:

   ```bash
   cd /ros2_ws && colcon build --symlink-install --packages-select <pkg>
   colcon test --packages-select <pkg> --event-handlers console_direct+ && colcon test-result --verbose
   bash tools/ci/guards.sh                              # from the repo root, on the host (needs the git repo)
   ```

5. Commit with a **conventional message**: `feat(driver): publish camera_info` (the hook rejects anything else).
6. Open a PR with `Closes #<issue#>`, fill in the checklist; CODEOWNERS requests the review and the ticket moves to **In review**.
7. **Squash merge** once CI is green and the PR is approved: the ticket closes and moves to **Done**, the branch is deleted.

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
- No magic numbers: constants in `so101_common`, tunables as ROS parameters (YAML in `so101_bringup`); both packages are planned at roadmap step 0.3.
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
# resolve conflicts, then open a "chore: sync upstream" PR
```

## 8. Tickets and the GitHub Project

All planned work lives in **one GitHub Project** (board + backlog) linked to this repository. Setup, fields, views and automations: `docs/project/15` §12.

- **One ticket = one PR** (a few hours to two days). Bigger? Split it. Tickets are grouped under one **epic per roadmap stage** (sub-issues; *phase 1 / phase 2* are the brief's parts, `docs/project/12`) and carry the roadmap step in their title: `[1.3a] Publish camera_info and use the optical frame`.
- **Statuses**: Backlog → **Ready** → In progress → In review → Done. At most **two tickets In progress per person**.
- **Fields**: Lane (A, B, C, Shared, `docs/project/09` §1.2), Priority (P0 critical path · P1 needed for the final defense · P2 optional), Size (XS < 2 h · S ≤ ½ day · M ≤ 1 day · L ≤ 2–3 days). Dependencies use GitHub's native **blocked by** relationship, not labels.
- **Labels**: `task`, `bug`, `epic`, `safety` (moves the real arm: [SAFE] review, sim first, `docs/project/07` §7), `staff-question`, and one or more `area:*`.

**Anatomy of a ticket** (the *Task* issue form): Roadmap step · Goal · Scope · Out of scope · **Done when** · Files and interfaces · References · Review focus. Write it so that **a teammate or an AI agent can do it without asking**: exact paths, checkable criteria (a command, a test, a threshold), what not to touch.

**Definition of Ready** (Backlog → Ready): every *blocked by* ticket is Done; *Done when* is checkable; files and interface impact are named; open questions answered or listed in the ticket.

**Definition of Done** (applies to every ticket, on top of its own *Done when*):

- [ ] Merged through a reviewed PR with green CI (lint, guards, build, tests).
- [ ] Rules of §3 respected (only `/joint_command`, no ground truth, no provided extrinsics, kit untouched).
- [ ] A test covers the new behavior; measurements reported in the PR (and in `results/` for milestones).
- [ ] Docs, parameters tables and ADRs updated in the same PR when they changed; the reviewer can explain the change.

**Working with an AI agent**: give it the issue; it reads `AGENTS.md` (commands, prohibitions, traps; `CLAUDE.md` imports it) and this file. The ticket body is the specification; *Out of scope* is binding; if a referenced document is ambiguous, the agent decides, says so in the PR and updates that document in the same PR. The author stays responsible for the PR (C-07).
