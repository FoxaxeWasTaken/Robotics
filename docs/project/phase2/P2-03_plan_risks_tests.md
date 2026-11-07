# P2-03 — Risks, tests and questions

> Authors: [CP] Project lead, [QA] Quality · Reviewers: all experts
> Requirements: C-54 · Decisions: ADR-034 to ADR-038 · Roadmap: [`P2-04`](P2-04_roadmap.md) (stage 7 of `09`)

## 1. Plan

Moved to its own roadmap: [`P2-04_roadmap.md`](P2-04_roadmap.md) (steps P2.0 to P2.9, same rules as `09`).

## 2. Risk register

Same scale as `10` (P = probability, I = impact, 1 to 3; Crit. = P × I).

| ID | Risk | P | I | Crit. | Warning sign | Mitigation | Owner |
| --- | --- | --- | --- | --- | --- | --- | --- |
| P2-R-12 | Token contact unstable in the merged scene: jaws squeezing through, tokens creeping or going through the slot bottom | 2 | 3 | **6** | Default contacts already do all of it [MES-63] | Working set of `P2-02` §7; spike #76 on one layout; cut-off: token welded to the jaw (`P2-04` §4) | CTRL |
| P2-R-13 | The review delay: 8 PRs, each up to 15 h, one approval, a push dismisses it | 3 | 2 | **6** | A PR open for more than a day | Small grouped PRs, reviewers warned, stacked drafts; last feature PR by 2026-11-17 | CP |
| P2-R-14 | The numbers come from separate prototypes: scene, pose, reading were measured on slightly different layouts, without ROS for the moves | 2 | 3 | 6 | #76 or #84 far from [MES-63], [MES-64] | #76 merges them first; rerun after `release_pose` replaces the prototype's IK | KIN |
| P2-R-15 | Hold through ROS unproven: `give_token` and the jaws only tested in-process | 2 | 2 | 4 | Tokens lost in #84 | Cut-off 2026-11-12: a sim-only weld on the pinch | CTRL |
| P2-R-16 | Perception across error above the slot's margin | 1 | 3 | 3 | Drops below 95 % with the pose, not with the config | The guide (capture −10/+6 mm [MES-63]); cut-off 2026-11-14: targets from `connect4_grid.yaml`, markers as a check | VIS |
| P2-R-17 | GitHub Actions minutes: about 2,500 of 3,000 this month (estimated, the billing API is out of reach). Out of minutes, no required check runs, and nothing merges | 2 | 3 | 6 | Settings → Billing | Check it now; the game job is separate; dispatch fewer nightlies | OPS |
| P2-R-18 | 8/10 is a noisy gate: at p = 0.90 it passes 93 % of the time (binomial) | 1 | 2 | 2 | A red night on a good commit | `release.sh` reads the newest report on the SHA: a second dispatch needs no new commit | QA |
| P2-R-05 | Tokens jam or jitter in the sim slots | 1 | 2 | 2 | Jammed tokens in the spike | 378/378 drops on the working set [MES-63] | CTRL |
| P2-R-09 | Python engine too slow for a useful depth | **closed** | | | | Depth 10 to 12 in 1 s [MES-66] | DEC |
| P2-R-04 | Phase 2 eats phase-1 time | **closed** | | | | Phase 1 is released (`v1.3.0`) | CP |

**On the real arm (`v2.1.0`)**, with the findings of `P2-02` §8:

| ID | Risk | P | I | Crit. | Warning sign | Mitigation | Owner |
| --- | --- | --- | --- | --- | --- | --- | --- |
| P2-R-01 | Real drops miss the slot (static error, sag, backlash) | 3 | 3 | **9** | Real spike out of its limits | Settle trim at rest, then release threshold; depth guide; roll offset (`P2-02` §8) | KIN, CTRL |
| P2-R-02 | Board misread (lighting, glare, background) | 2 | 3 | 6 | Frequent `FIX_BOARD` | Neutral background, human on the camera side, read with the arm out of view | VIS |
| P2-R-03 | Calibration zero ≠ URDF zero on the real arm | **settled** | | | | `joint_zero_offsets`, measured per arm [MES-55] | CTRL |
| P2-R-06 | Token badly placed at the handover (off-center, not held, slips) | 2 | 2 | 4 | Drops off by one column, tokens lost | Held-token check; printed jaw pads with a recess | CTRL |
| P2-R-07 | A finger pinched, a hand in the workspace | 2 | 3 | 6 | — | Slow close, operator on "token given", latched stop, dummy test; blocking findings of `P2-02` §8 | SAFE |
| P2-R-10 | The purchased grid does not fit the arm | 1 | 2 | 2 | Far column out of reach | Riser and radius from the measurements (#88) | KIN |
| P2-R-11 | Wrist-camera cable snags, or the mount shifts | 2 | 1 | 2 | Image shifted in the check pose | Only if the wrist camera is kept (`P2-02` §8) | CTRL |
| P2-R-08 | Interface change late or rejected | **closed** | | | | `/brain/drop` needs no refactor (ADR-037 Update) | ARC |

## 3. Tests and measurements

| Level | What | Tool | Target |
| --- | --- | --- | --- |
| Unit | `game.py`: wins in the 4 directions, draw, full column, known positions, 100 games against random at depth 4 | pytest | P2-04 |
| Unit | `turns.py`: every transition and failure row; a random walk of events: the arm moves in `MOVE` only | pytest | P2-05 |
| Unit | `board.py`: synthetic boards, reference PNGs rendered from the scene | pytest | P2-01 |
| Unit | `release_pose`: the 7 columns, token centre within 1 mm, jaw error < 0.2°, tilt 70°; unreachable → `None` | pytest | ADR-035 |
| Package | Each package with its own fakes: `so101_driver` (services), `so101_brain` (`/brain/drop`, fake driver), `so101_connect4` (fake camera, fake brain). colcon's coverage counts per package: a bringup test covers no driver code | pytest, `colcon test` | Quality gate, 80 % on new code |
| Integration (sim) | Scene: tick and image rate with 42 tokens; a render finds the 4 markers | `so101_bringup/test` | #79 |
| Integration (sim) | Board reader against the scene's truth (tests only, C-06): 10 states in `colcon test`, 50 once as a MES | `so101_bringup/test` | P2-01 |
| Integration (sim) | One drop in `colcon test`; 50 drops once as a MES, random columns; 10 tries without a token | `so101_bringup/test`, `experiences/` | P2-02, P2-03 |
| End-to-end (sim) | Game bench: N games against a scripted human, random or engine | `evaluate --games` (`08`) | P2-06 |
| Spike | 100 drops, 100 with ±5 mm TCP offset, capture sweep, full stacks | `experiences/connect4/scene/` | Go / no-go, #76 |
| Real (`v2.1.0`) | 30 drops, 30 handovers, 20 board photos, 3 filmed games; guide vs camera if the wrist camera is kept | manual protocol, results in `results/` | `P2-01` real parts |

### 3.1 Phase 2 in the CI

| When | What |
| --- | --- |
| Every PR | The 7 required checks, unchanged in name: lint, guards (extended to `so101_connect4`, plus no `/joint_command` in the game and no `/sim/` in the pipeline), both builds and tests, phase-1 E2E. New: `E2E sim (1 game)`, a short seeded game, **not required**, so a flaky game cannot freeze every PR in the last week |
| Each night `main` changed | The phase-1 bench, then 10 games on a restarted stack; one verdict line, `Target ≥ 90 % and ≥ 8/10 games: met.`, read by the report and by `release.sh`; SonarQube Cloud, coverage per package, trends |
| Release | `release.sh` needs the bench on the exact commit, the gate and the CHANGELOG (`15` §6). The gate is read **before** the last PR (#227) |

Ten games last about 15 to 25 min (estimated, [MES-64]: 5 to 8 s a move, 9 robot moves a game), in a job of 120 min that uses 19 today. Not worth doing here: CI or Sonar badges, a Sonar scan on PRs, path filters, mypy, mutation tests on the game, a matrix.

Who sees it: the repository is private and the school copy has no Actions, Issues or Releases, so the visible proof is in the tree: `.github/workflows/`, `results/phase2_<date>.md`, the README (#234).

Every result enters the register of `../11_cross_review_and_measurements.md` as a MES, like phase 1.

## 4. Questions for the teaching staff

Answered on 2026-10-16, with the questions of `../README.md`. *Team* marks what the team settled itself.

1. **Riser** (*team*): the arm may be placed as the team wants, riser included; on the riser it goes through the `07` §3 commissioning again (`P2-02` §2.4).
2. **Cameras** (*team* and staff): a grid webcam and a wrist camera are both fine, lent by the school. Any USB webcam works for the grid; the wrist camera is probably the WowRobo 2 MP USB module, with a printable mount (STL on Teams); availability per campus: the staff will confirm (ADR-038 update).
3. **Real-arm sessions** (*team*): access whenever needed.
4. **Taught poses** (staff): reuse phase 1 as much as possible; the fallback is accepted if the PR and the defense explain why and show that the computed poses were tried and fixed first (`P2-02` §2.4).
5. **3D printer** (*team*): available on request.
