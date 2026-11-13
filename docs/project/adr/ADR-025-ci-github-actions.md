# ADR-025 — CI: GitHub Actions with the project's Docker images

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | OPS → QA, ARC | C-05, C-06, C-23, C-42 |

## Context

The key dependencies (MuJoCo, SO101Sim, minimal LeRobot, numpy<2, Pinocchio, ikpy) only live in our Dockerfiles. The repository is on GitHub.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| No CI (manual tests) | Zero cost | Regressions and brief violations slip through |
| `ros-tooling/setup-ros` + `action-ros-ci` (native ROS on the runner) | ROS standard | All our dependencies to reinstall outside Docker: dev/CI drift |
| `industrial_ci` | Mature, multi-distro | Same parity problem; specific config |
| **Our images (`docker/build-push-action` + GHA cache), tests inside the container** | **Strict parity** with the dev workstation; headless OSMesa sim | ~10 min first build; 4.2 GB image |

## Decision

`ci.yml` (PRs and `main`): `lint` → `guards` → `control` and `perception` (image build with cache, then `colcon build/test`; `perception` skipped while `COLCON_IGNORE` exists) → `e2e-smoke` (3 episodes, `vars.E2E_ENABLED`). `nightly.yml`: Mermaid rendering + links (lychee), 50-episode bench. Every job is **opt-in** (`vars.CI_ENABLED`) so nothing runs on the school mirror. `concurrency` with cancellation, `contents: read` permissions.

## Consequences

- Required checks on `main`: `lint`, `guards`, `control`, `perception`.
- Private repo: watch the 2,000 min/month quota; space out the nightly.

## Evidence / validation

actionlint 1.7.12: 0 error; the `control` job command run locally: build OK, 6/6 tests [MES-23].

## Reconsider if

Minute quota exceeded (self-hosted runner) or build too long (publish the image on GHCR and pull it).

## Update (2026-10-09)

The job command as first written (`colcon test --packages-skip so101_perception`) exited 1: under Python 3.12 a package without tests fails with "NO TESTS RAN". The 6/6 of [MES-23] came from the driver package alone. The job now tests only the packages that have a `test/` folder; replayed: exit 0, 6/6. Required checks are selected by their displayed job names (`14` §10).

## Update (2026-10-15)

Actions pinned by commit SHA and updated by Dependabot (7-day cooldown); `persist-credentials: false` on every checkout; runs on `main` are never cancelled; `ament_xmllint` in the control job; required checks: the four jobs plus `PR title (conventional commit)` (ruleset "main protection").

## Update (2026-10-30, #189)

Since this decision (`14` §5):

- **no CI on a push to `main`** (#153): the ruleset requires a PR up to date with `main`, so `main` gets the tree its CI tested; `main` pushes run the mirror only;
- a **draft** or a **docs-only** PR runs lint, the guards and the PR title only (#146);
- `E2E sim (3 episodes)` is a **required check**, 3 successes out of 3 (#156);
- the nightly runs its bench and its `quality` job only when `main` changed (#146), and reports both on "Nightly bench results" (#151, #179);
- the quota is GitHub Pro's 3,000 min/month, not 2,000.

## Update (2026-11-13, phase 2, #233)

One more job on each PR, `E2E sim (1 game)`: a full Connect Four game on the compose stack in game mode, 44 billed minutes a push with it. Not a required check until its runs are all green (decided on 2026-11-19). The nightly plays 10 games after its 50 episodes and gives one verdict for both, which `tools/release.sh` reads (`14` §5, §6).
