# 14 — Quality pipeline: tests, lint, continuous integration

> Authors: [QA] Quality & tests, [OPS] DevOps · Reviewers: [ARC], [CP], [CTRL]
> Decisions: ADR-019, ADR-024, ADR-025, ADR-026, ADR-030, ADR-033 · Files installed at the repository root: `.github/`, `pyproject.toml`, `.pre-commit-config.yaml`, `tools/ci/`
> **Everything described here was executed**: ruff, markdownlint, actionlint, mermaid-cli, pre-commit, `colcon test` [MES-22], [MES-23], [MES-24].

## 1. What the pipeline guarantees

| Guarantee | Mechanism | Requirement served |
| --- | --- | --- |
| The project **follows the brief's rules**, permanently | Automated guards (`tools/ci/guards.sh`) | C-05, C-06, C-23 |
| No regression is merged into `main` | Mandatory CI on every PR (protected branch) | C-42 and all |
| Homogeneous code whoever the author | ruff (lint + format), pre-commit | C-50, C-53 |
| Announced performance is **measured**, not assumed | Component tests (rates), E2E bench | C-11, C-41, C-42 |
| Documentation stays correct (diagrams render, links alive) | markdownlint, mermaid-cli, lychee | C-53 |
| Readable history attributable to each member | Conventional commits, reviewed PRs | C-07 |

## 2. Overview

```mermaid
flowchart LR
  subgraph L["Developer workstation"]
    E["Editor<br/>ruff on save"] --> PC["pre-commit<br/>(git commit)"]
    PC --> CM["commit-msg<br/>conventional commit"]
  end
  subgraph G["GitHub (private repo)"]
    PR["Pull request"] --> CI["PR CI<br/>lint · guards · build · tests"]
    CI --> RV["Review<br/>CODEOWNERS"]
    RV --> M["main (protected)"]
    M --> CIM["CI on main"]
    M --> N["Nightly<br/>50-episode bench · docs"]
    M --> T["Tag vX.Y.Z<br/>Release + report"]
    M --> MI["mirror.yml<br/>to the school repo"]
  end
  CM -->|git push| PR
```

### 2.1 Feedback loops

| Loop | When | Target duration | What it catches |
| --- | --- | --- | --- |
| Editor (ruff, markdownlint) | While typing | Instant | Style, imports, trivial errors |
| `pre-commit` | Every `git commit` | < 10 s | Lint, format, invalid YAML/XML/TOML, large files, private keys, commit message |
| Local tests (`colcon test --packages-select …`) | Before pushing | < 1 min per package | Unit and component regressions |
| PR CI | Every push to a PR ready for review (a draft or a docs-only change: lint, guards and PR title only, §5.4) | ≈ 17 min (cached) | All of the above + project-rule guards + clean build + short E2E |
| Nightly | Every night or on demand | 30–60 min | 50-episode success rate, diagrams, external links, new course commits |
| Release | Every tag | — | Frozen evaluation report, release notes |

## 3. Check matrix: who checks what, and where

| Check | Editor | pre-commit | PR CI | Nightly | Tool |
| --- | --- | --- | --- | --- | --- |
| Python lint | ✅ | ✅ | ✅ | | ruff |
| Python format | ✅ | ✅ | ✅ | | ruff format |
| Markdown lint | ✅ | ✅ | ✅ | | markdownlint-cli2 |
| Valid YAML / TOML / XML | | ✅ | ✅ | | pre-commit-hooks |
| Files > 1 MB blocked, private keys | | ✅ | ✅ | | pre-commit-hooks |
| LF line endings | ✅ (editorconfig) | ✅ | ✅ | | mixed-line-ending + `.gitattributes` |
| Conventional commit message | | ✅ (commit-msg) | ✅ PR title (= the squash commit) | | conventional-pre-commit · `pr-title.yml` |
| Workflows: syntax and security (pinned actions, token scope, injection) | | ✅ | ✅ | | actionlint · zizmor |
| Shell scripts · Dockerfiles | | ✅ | ✅ | | shellcheck (warnings) · hadolint (`.hadolint.yaml`) |
| Secrets in the whole history | | | ✅ | | gitleaks |
| `package.xml` against the ROS schema | | | ✅ | | `ament_xmllint` (control job) |
| **Project-rule guards** (C-05, C-06, C-23) | | | ✅ | | `tools/ci/guards.sh` |
| `colcon` build (control and perception images) | | | ✅ | | Docker + colcon |
| Unit tests | | | ✅ | | pytest via `colcon test` |
| Component tests (real node, headless sim) | | | ✅ | | launch_testing |
| Short E2E (3 episodes) | | | ✅* | | `ros2 run so101_bringup evaluate` |
| **Full E2E (50 episodes, randomized cube and box)** | | | | ✅* | same |
| Rendering of every Mermaid diagram | | | | ✅ | mermaid-cli (`tools/ci/check_mermaid.sh`) |
| External links | | | | ✅ | lychee (`.lycheeignore`: the private repository) |
| New commits in the course repository: one issue | | | | ✅ | `tools/ci/upstream_check.sh` (`15` §2) |
| Test of that watch, offline | | ✅ | ✅ | | `tools/ci/test_upstream_check.sh` (local hook) |

\* enabled on 2026-10-23, once the vertical slice worked (4.3a, #139): repository variable `E2E_ENABLED=true`. Every CI job also requires `CI_ENABLED=true` (opt-in, so nothing runs on the school mirror, `16`).

## 4. Tool choices (options considered)

| Need | Options | Choice | Why |
| --- | --- | --- | --- |
| Python lint and format | flake8 + black + isort · `ament_flake8`/`ament_pep257` (ROS ament_lint) · **ruff** | **ruff** (ADR-024) | One tool, 10–100× faster, autofix, configured in `pyproject.toml`; ament_lint imposes per-package config and redundant lint tests |
| Hook manager | husky (Node) · lefthook · **pre-commit** | **pre-commit** | Python ecosystem; **same configuration locally and in CI** |
| Commit messages | commitlint (Node) · **conventional-pre-commit** | **conventional-pre-commit** + a PR-title check | No Node install for the team. Squash merge writes the **PR title** on `main`, so a few lines of shell check it (no third-party action, no token) |
| Workflow security | none · **zizmor** · CodeQL for Actions | **zizmor** in pre-commit; actions pinned by commit SHA, updated by Dependabot (7-day cooldown) | CodeQL needs GitHub Team + Code Security on a private repository; zizmor is free and local |
| Secrets | `detect-private-key` · GitHub secret scanning · **gitleaks** | **gitleaks** on the whole history in the lint job | Secret scanning and push protection need a paid plan on a private repository; free once public |
| Code quality platform | **[SonarQube Cloud](https://sonarcloud.io/project/overview?id=FoxaxeWasTaken_Robotics), on trial since 2026-10-27** (#61; members of the SonarCloud organization only) · CodeQL | Analyses of `main` by the nightly's `quality` job, with the coverage of every package's tests; **not a required check**. Decide by 2026-11-10: keep, add PR analyses (a scan after the control job), or drop. CodeQL and secret scanning when the repository goes public | Sonar adds a gate on new code (coverage, duplication, complexity) that ruff does not; no CI on `main` since #153, so the nightly runs its analyses |
| Typing | mypy · pyright · none | **Staged**: not blocking in phase 1, then `mypy` on pure modules (`so101_common`, `kinematics`, `fsm`, `geometry`) once stable | rclpy nodes have few stubs; maximal gain on pure logic |
| Tests | unittest · **pytest** · **launch_testing** | **pytest + launch_testing** through `colcon test` | Already in the image (pytest 7.4.4, launch_testing, launch_testing_ros) [MES-23] |
| Property tests | hand-written loops · **Hypothesis** | **Hypothesis** (apt `python3-hypothesis`, 0.3d) | Hundreds of generated inputs, edge cases first (bounds, infinities); a failure shrinks to a minimal example. First use: `so101_common/test/test_robot.py` |
| Test isolation (DDS) | `ROS_DOMAIN_ID` set by hand · **colcon-ros-domain-id-coordinator** | **The coordinator** (apt, 0.3d) | `colcon test` gives each package a free domain, never the current one: a test sees neither a stack on the team's domain nor a parallel run. Measured with a stack on 42 and two parallel driver test runs: domains 18 and 59, 30 Hz each, all green; without it, 82 Hz received and one failure |
| Coverage | pytest-cov · Codecov / Coveralls (external services) · none | **pytest-cov** (`colcon test --pytest-with-coverage`) and a **non-blocking comment on each PR** by python-coverage-comment-action (no external service), target ≥ 80 % | Implemented at 0.3d (apt `python3-pytest-cov`). Paths stored relative to the repository (`pyproject.toml`); the launch tests count, since pytest-cov follows the node subprocess. A threshold as a required check comes later |
| Markdown | markdownlint-cli2 · remark | **markdownlint-cli2** | Same rules as the VS Code extension |
| Diagrams | Exported images · **Mermaid** in Markdown | **Mermaid, rendering checked in CI** (ADR-030) | Rendered natively by GitHub, versionable; broken syntax is detected |
| CI | GitLab CI · Jenkins · **GitHub Actions** | **GitHub Actions** (ADR-025) | Repo on GitHub, free for public repos, built-in Docker cache |
| CI environment | `ros-tooling/setup-ros` (native ROS on the runner) · `industrial_ci` · **our Docker images** | **Our images** | **Strict parity** with the dev workstation: mujoco, SO101Sim, numpy<2… only live in our Dockerfiles |

## 5. Pull-request CI

### 5.1 Job graph

```mermaid
flowchart LR
  A["lint<br/>pre-commit --all-files<br/>+ draft or docs only? (§5.4)<br/>≈ 1 min"] --> C
  B["guards<br/>C-05 · C-06 · C-23<br/>≈ 10 s"] --> C
  A --> P
  B --> P
  C["control<br/>image build (cache) + colcon build/test<br/>≈ 5–10 min"] --> E
  P["perception<br/>image build + perception tests<br/>≈ 3–5 min"] --> E
  E["e2e-smoke<br/>3 sim episodes<br/>(PRs, if E2E_ENABLED)"]
```

| Job | Fails when… | Artifacts |
| --- | --- | --- |
| `lint` | Also decides whether `control`, `perception` and `e2e-smoke` run (§5.4). Fails when a ruff, markdownlint, actionlint, zizmor, shellcheck or hadolint rule is violated, a file is not formatted, a YAML is invalid, a file exceeds 1 MB, gitleaks finds a secret in the history | Diff shown (`--show-diff-on-failure`) |
| `PR title` (`pr-title.yml`) | The PR title is not `type(scope): description` | — |
| `guards` | brain or perception imports the backend; ground truth or provided extrinsics appear in the pipeline | Offending lines |
| `control` | `colcon build` fails or a test fails (`colcon test-result --verbose`); only packages with a `test/` folder are tested, one at a time (`--executor sequential`): in parallel, the brain's IK property tests starved the driver's camera test (2.7 Hz on the 4-vCPU runner) | Each package's `pytest.xml` |
| `perception` | Same for the perception packages (separate build, ADR-002); **skipped until `COLCON_IGNORE` is removed** (step 3.1) | — |
| `e2e-smoke` | PRs only (§5.4). The stack is not ready within 15 min (`tools/ci/wait_for_stack.sh`), or the bench stops (exit 2: no ground truth, no brain pose). Fewer than 3 successes out of 3 fail it too (`evaluate --min-success 3`, #156): it replays the same three scenes each time (driver seed 0, a fresh stack), all three visible to the camera (MES-12), and passed 11 runs out of 11 after #141 and #144. The 80 % per episode of random scenes does not apply to these three; the nightly's 50 random episodes stay informative (#151). On failure, the job prints `docker compose ps -a` (an exited container shows there) and the last 150 log lines of `control` and `perception`, then the nodes a fresh process finds and whether `/joint_states` and `/end_effector_pose` reach it (#141). The stack runs with `FASTDDS_BUILTIN_TRANSPORTS=DEFAULT`, as the control tests: under the compose default `LARGE_DATA`, a fresh process missed the driver or the brain for 60 s in 2 of 5 runs on the runner. | `eval.csv` |

### 5.2 Technical notes

- **Docker cache**: `docker/build-push-action` with `cache-from/to: type=gha`. First build ≈ 10 min (apt + pip), then 1–2 min if the Dockerfiles did not change.
- **Runner disk**: the `control` image is 4.2 GB, fine on `ubuntu-24.04`; if space runs out, add a cleanup step (remove preinstalled SDKs) before the build.
- **Headless**: `MUJOCO_GL=osmesa`: sim and rendering run on CPU, no GPU or X11 [MES-02], [MES-03].
- **`concurrency`**: a new push cancels the previous run of the same PR. No CI on `main` since #153 (§5.4).
- **Supply chain**: every action pinned by commit SHA (version in a comment), updated by Dependabot after a 7-day cooldown; `persist-credentials: false` on every checkout (zizmor).
- **Dependabot PRs**: GitHub runs them like fork PRs, and repository variables may not reach them; a skipped required check would count as passed. The jobs therefore also run when the PR author (`github.event.pull_request.user.login`, not the spoofable `github.actor`) is `dependabot[bot]`, **in this repository only** (`github.repository`): the school copy mirrors `dependabot.yml`, and Dependabot ran there on 2026-10-16 (`16` §4).
- **Minimal permissions**: `contents: read`.
- Validated by **actionlint 1.7.12: 0 error** on `ci.yml`, `nightly.yml`, `mirror.yml` [MES-23].

### 5.3 Minute budget

GitHub bills each job rounded up to the minute. Measured on 2026-10-23 (#146), from the jobs' durations:

| Run | Billed |
| --- | --- |
| PR push that changes code | 22 min: `control` 12, E2E 4, `perception` 2, lint, guards, coverage comment and PR title 1 each |
| Draft or docs-only PR push (§5.4) | 3 min: lint, guards, PR title |
| Push to `main` (a merge) | 1 min since #153, the mirror only (16 before: a PR push without the E2E job, the coverage comment and the PR title) |
| Nightly, `main` unchanged | 5 to 6 min: docs 3–4, course watch 1, `main-changed` 1 |
| Nightly, `main` changed | + about 20 min: build and 50 episodes (estimate, the first scheduled bench is the night of 2026-10-23) |

The table below is the plan from 2026-10-15:

| Repo | GitHub quota (free plan) | Estimate | Consequence |
| --- | --- | --- | --- |
| Public | Unlimited | — | Daily nightly without constraint |
| Private, GitHub Pro (our plan) | 3,000 min/month | Measured on 2026-10-15: a cached PR run ≈ 3.4 min of jobs, ≈ 6 billed (each job rounded up to the minute); first build ≈ 10 min. 100 PR pushes ≈ 600 min, leaving room for a nightly bench (≈ 45 min) most days | Daily nightly once the bench exists; watch Settings → Billing |

### 5.4 What runs when (#146)

| Event | lint, guards, PR title | `control`, `perception`, coverage comment | `e2e-smoke` |
| --- | --- | --- | --- |
| PR push that changes code | Run | Run | Run |
| PR push that only touches documentation | Run | Skipped | Skipped |
| Push to a **draft** PR | Run | Skipped | Skipped |
| **Ready for review** (and every later push) | Run | Run, unless documentation only | Run, unless documentation only |
| Push to `main` (a merge) | Nothing since #153: the mirror only (`mirror.yml`) | Nothing | Nothing |

- **No CI on `main`** (#153): the `main protection` ruleset requires a PR to be **up to date with `main`** before it merges ("Require branches to be up to date before merging", since 2026-10-24), and merges by squash. The PR's CI tests the merge of its branch with `main`, so `main` gets exactly the tree that CI tested: a run after the merge would repeat it (16 billed minutes per merge, about 1,500 a month at the pace of 2026-10-16 to 23). The cost moves to the branch updates, which the team already made before each merge. The nightly still covers `main` every night it changes. The Docker layer cache of `main` (written by those runs) stops being refreshed: measured on 2026-10-24, a cached build of the control image takes 89 s, both images without cache 179 s, so a PR loses at most about 40 s after a Dockerfile change; not worth a cache export in the nightly.
- **Documentation only** (`tools/ci/docs_only.sh`): every changed file is a `*.md`, a PDF or an image under `docs/`, or an issue form (`.github/ISSUE_TEMPLATE/`). The list is `git diff --name-only --no-renames` of the PR's merge commit against its base: a file moved from code to a `.md` still counts as code. Code, tests and their data (the perception's reference images), experiments, workflows, Docker, launch, config and tooling files always run everything, and so does an empty list.
- The `lint` job decides, not a separate job: `lint` is a required check, so a failed decision blocks the PR instead of letting it merge without its tests, and no extra job is billed.
- **`E2E sim (3 episodes)` is a required check** since #156: a failed pick & place blocks the merge. A failed run gets one rerun of the failed job at most; a second failure is a `bug` issue (CONTRIBUTING §5).
- **A skipped required check counts as passed**: a docs-only PR merges with `control`, `perception` and `e2e-smoke` skipped. A draft cannot be merged, and marking it ready reruns everything.
- Tested without GitHub by `tools/ci/test_ci_skips.sh` (pre-commit, when one of the scripts changes).

## 6. Nightly

```mermaid
flowchart LR
  S["cron 03:00 UTC<br/>or workflow_dispatch"] --> D["docs<br/>mermaid-cli + lychee"]
  S --> MC["main-changed<br/>main moved since the last bench?"]
  MC -->|"yes, or workflow_dispatch"| Q["quality<br/>every test with coverage, trends.csv,<br/>SonarQube Cloud"]
  MC -->|"yes, or workflow_dispatch"| V["evaluation<br/>compose up control+perception+viz<br/>50 randomized episodes"]
  V --> R["artifacts<br/>eval.csv + failure rosbags (14 days)"]
  V --> B["report<br/>run summary + one comment on<br/>Nightly bench results"]
  B --> H["Humans: success-rate trend,<br/>failure-cause analysis"]
  S --> W["upstream<br/>course main vs .github/upstream-sha"]
  W --> I["one issue<br/>Course repository: new commits"]
```

- **The quality job** (#60, #61), when `main` changed, as the bench: every package's tests with coverage (perception's too, in the control image), the IK benchmarks (`so101_brain/test/test_benchmark.py`, pytest-benchmark) and the driver's launch test (its `TREND` line), then `tools/ci/trends.py` writes `trends.csv` (artifact `trends-<run>`, 90 days; values in `08` §2), and the SonarQube Cloud scan (`sonar-project.properties`, secret `SONAR_TOKEN`). No threshold: CI runners are too noisy for a gate.
- The bench writes one CSV row per episode: poses, final state, duration, failure cause (`08` §3).
- **The report** (#151, `tools/ci/bench_report.sh`, tested by `test_bench_report.sh`): each night the bench runs, its summary line (success rate, p50/p95 duration, failure causes), the commit of `main`, the target (≥ 90 %, informative: the nightly does not fail under it) and the run link go to the run summary and to one comment on the issue "Nightly bench results". A bench that stopped, or never started, comments that too. Skipped nights add nothing. The first report created it (#155), labeled `report` so the project views leave it out (it is not a work item), then pinned by hand; only this job can write issues.
- The bench runs only when `main` changed (#146): `main-changed` (`tools/ci/evaluation_due.sh`) finds the newest nightly run whose bench gave a verdict, success or failure, and skips the bench when that run was on the same commit. Skipped nights are passed over, and so are cancelled or timed-out benches. Any doubt runs it: no verdict among the last 30 runs, or the API failing; the bench is then skipped only on an explicit `false`. A failed bench is not rerun on the same commit: it was reported that night, and `workflow_dispatch` always runs it.
- On failure, the evaluation job prints the stack state, the logs and the links still alive, and runs on the `DEFAULT` transports, as `e2e-smoke` (#141).
- The `upstream` job (#137) compares the course's `main` with `.github/upstream-sha`. When they differ, it opens one issue listing the new commits and their files, or updates the one already open. It runs without `E2E_ENABLED`, and its token can only write issues.
- Reports of important milestones are **versioned by hand** in `results/` (small CSV/MD), never the rosbags (ADR-029).

## 7. Tests: conventions (ADR-026)

### 7.1 Where should a test go?

```mermaid
flowchart TD
  Q{"Does the behavior depend on ROS<br/>(topics, timers, services)?"}
  Q -- no --> U["UNIT pytest test<br/>pure module: fsm, kinematics, geometry, detection, so101_common<br/>fast, deterministic"]
  Q -- yes --> Q2{"Is the real node and the sim needed?"}
  Q2 -- "no (logic can be extracted)" --> X["Extract the logic into a pure module, then unit test"]
  Q2 -- yes --> Q3{"A single node?"}
  Q3 -- yes --> C["COMPONENT test launch_testing<br/>rate, partial command, services"]
  Q3 -- "no (full stack)" --> E["E2E bench<br/>N-episode evaluation"]
```

### 7.2 Rules

| Rule | Detail |
| --- | --- |
| Location | `ros2_ws/src/<pkg>/test/`: `test_*.py` (unit), `test_*_launch.py` (component) |
| Declaration | `setup.py`: `tests_require=["pytest"]`; `package.xml`: `<test_depend>python3-pytest</test_depend>`, `<test_depend>launch_testing_ros</test_depend>` (done for `so101_driver`) |
| Running | `colcon test --packages-select <pkg> --event-handlers console_direct+` then `colcon test-result --verbose` |
| Determinism | Fixed seed (`seed`); no test-order dependency; no network |
| Timing tests | Measure over ≥ 3 s **after** a 1 s warm-up; explicit tolerances; values as parameters, not hard-coded |
| ⚠ Masked by the sim | MuJoCo already clamps setpoints to `ctrlrange`: a sim test does **not** prove the driver clamps (C-15). The clamp is tested at **unit** level on the command-merge function |
| Ground truth | Allowed **only** under `test/` and in the bench (C-06), enforced by the guards |
| Flaky tests | Forbidden in `main`: `@pytest.mark.flaky_quarantine` marker, excluded from CI, issue opened with a 1-week fix deadline |
| Test data | Small reference images (< 200 KB each) in `test/data/`, generated by a versioned script |

### 7.3 Example tests and what they revealed [MES-22], [MES-24]

Two tests of the bootstrap prototype (outside the repository since 2026-10-15, ADR-039), kept here as an example:

| Test | Type | Early driver | Merged driver |
| --- | --- | --- | --- |
| `test_gripper_conversion.py` (5 cases) | Unit | ✅ 5/5 | ✅ 5/5 |
| `test_driver_launch.py::test_2_partial_command…` | Component | ✅ | ✅ |
| `test_driver_launch.py::test_1_joint_states_rate` | Component | ❌ **4.7 Hz** instead of ≥ 20 (`ros2 topic hz`: 9 Hz) | ✅ **30 Hz** |

**The pipeline did its job before it was even installed**: it detected a non-compliance with the brief (C-11) that manual testing did not show (30 Hz announced in the code). The fix is ADR-006 (camera in a dedicated thread rendering a copy). Total: 6 tests in ~9.4 s.

## 8. Project-rule guards

| Guard | Pattern searched | Where | Requirement |
| --- | --- | --- | --- |
| No backend outside the driver | `SO101Sim`, `SO101Follower`, `so101_sim`, `lerobot` | `so101_brain`, `so101_perception` | C-05 |
| No ground truth in the pipeline | `get_object_position`, `get_drop_box_position`, `ground_truth` | brain, perception (outside `test/`) | C-06 |
| No launch file enables ground truth | `gt_perception_stub`, `expose_ground_truth … True` | `*.launch.py` (outside `test/`) | C-06 |
| No provided extrinsics | `get_camera_extrinsics` | perception (outside `test/`) | C-23 |

`git grep --untracked` also covers files not committed yet when run locally.

## 9. Lint on the existing code [MES-23]

State of the code with the final configuration (ruff 0.16.10), before the cleanup:

| File | Issues | Nature |
| --- | --- | --- |
| `so101_perception/perception_node.py` (provided stub) | 8 | 7 unused imports, 1 import order |
| `so101_brain/brain_node.py` | 4 | Import order, `zip()` without `strict`, long line, list concatenation |
| `so101_driver/driver_node.py` | 1 | `zip()` without `strict` |
| `so101_driver/launch/driver.launch.py` (provided) | 2 | Long lines (comments) |
| **Total** | **15** (9 auto-fixable), **7 files to reformat** | |
| *Course-provided driver (HEAD after the 2026-10-08 pull)* | *0* | *Compliant* |

What was done so that CI is green from the first push **without diverging from upstream**:

- **Team code** (`driver_node.py`, `brain_node.py`): fixed and formatted (≈ 6 lines; the provided part of the driver is untouched).
- **Provided files still identical to upstream** (the launch files of `so101_brain` and `so101_driver`, `example_py_pkg`, `ros2_ws/.gitignore`, `__init__.py` in mode 755): **excluded** in `pyproject.toml` / `.pre-commit-config.yaml` rather than modified. Each file is listed by name, never by a glob: `ros2_ws/src/*/launch/*.launch.py` also skipped the team's `sim.launch.py`, `real.launch.py` and `perception.launch.py` until #161. Remove a file from the exclusion list when the team rewrites it.
- Course content (`core/`, `sim/`, `so101_description`, `docs/SUJET.md`, slides) is excluded from every check; `docs/SUJET.md` would otherwise raise 13 markdownlint issues [RC-30].
- `docs/` and `*.md` are excluded from ruff, which also formats Python blocks inside Markdown (doc snippets are deliberately compact).
- Rule excluded after review: `N803`/`N806` (upper-case variables): `R`, `T`, `K`, `J` denote matrices by robotics convention.

Result: **13/13 pre-commit hooks green on the whole repository**, guards green, 6/6 driver tests [MES-23]. The CI command itself exited 1 at the time (packages without tests count as failed under Python 3.12); fixed on 2026-10-09 and replayed: exit 0, 6/6.

For any future mass reformatting: one dedicated `style:` commit, then its SHA in `.git-blame-ignore-revs` (`git config blame.ignoreRevsFile .git-blame-ignore-revs` locally; GitHub reads the file automatically).

## 10. Setup

Files are **in place at the repository root** (workflows, hooks, configurations). GitHub settings, done on 2026-10-15 unless stated:

1. Replace `@member1/2/3` in `.github/CODEOWNERS` with the lane owners of `09` §1.2 (done, ticket 0.1d). CODEOWNERS is indicative: GitHub requests the owners' review, but the ruleset does not require it, so any member may approve any PR (team decision, 2026-10-23).
2. Every member: `pipx install pre-commit && pre-commit install --install-hooks -t pre-commit -t commit-msg` (plain `pip install` is refused on Ubuntu 24.04, PEP 668).
3. Private repo → Settings → Variables: `CI_ENABLED=true` (done) (CI and nightly are opt-in so they do not run on the school mirror, `16`).
4. Settings → Rules (or Branches): protect `main` (PR required, 1 approval, required checks **by their displayed job names**: `Lint (pre-commit)`, `Project rules (C-05, C-06, C-23)`, `Build + tests (control image)`, `Build + tests (perception image)`, `PR title (conventional commit)`, and `E2E sim (3 episodes)` since #156). A private repository needs GitHub Pro for this (`15` §12.5). Done on 2026-10-15 (ruleset "main protection", no bypass).
5. School mirror: `16` §5 (done; first run at the merge of PR #1).
6. Step 3.1: the `perception` job enables itself once `COLCON_IGNORE` is removed from the package.
7. Variable `E2E_ENABLED=true` once the vertical slice works (4.3a): done on 2026-10-23, after #139. The E2E jobs set `EXPOSE_GROUND_TRUTH=true` for the bench, then wait for the stack.
