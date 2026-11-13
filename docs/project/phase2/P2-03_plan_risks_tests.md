# P2-03 — Risks, tests and questions

> Authors: [CP] Project lead, [QA] Quality · Reviewers: all experts
> Requirements: C-54 · Decisions: ADR-034 to ADR-038, ADR-040 to ADR-042 · Roadmap: [`P2-04`](P2-04_roadmap.md) (stage 7 of `09`)

## 1. Plan

Moved to its own roadmap: [`P2-04_roadmap.md`](P2-04_roadmap.md) (steps P2.0 to P2.9, same rules as `09`).

## 2. Risk register

Same scale as `10` (P = probability, I = impact, 1 to 3; Crit. = P × I).

| ID | Risk | P | I | Crit. | Warning sign | Mitigation | Owner |
| --- | --- | --- | --- | --- | --- | --- | --- |
| P2-R-12 | Token contact unstable in the merged scene: jaws squeezing through, tokens creeping or going through the slot bottom | **closed** | | | | In the merged scene, through the driver: 42 tokens of 42 in their cell, full stacks at rest, the scene's tests fail on the default contacts [MES-67] | CTRL |
| P2-R-13 | The review delay: 8 PRs, each up to 15 h, one approval, a push dismisses it; the team works Thursday, Friday and the weekend only | 3 | 2 | **6** | A PR open for more than a day | Grouped PRs, the branch pushed early so the next ticket starts from it, reviewers warned; last feature PR by 2026-11-15 | CP |
| P2-R-14 | The numbers come from separate prototypes: scene, pose, reading were measured on slightly different layouts, without ROS for the moves | **closed** | | | | #76 merged them on one layout, 100/100; through the brain and ROS on the game's scene, 50/50 [MES-69] | KIN |
| P2-R-15 | Hold through ROS unproven: `give_token` and the jaws only tested in-process | **closed** | | | | Through the driver's tick and its `give_token`: held, 1.4 mm of slip after two moves at 1.5 rad/s; the closing effort tells a token from none, the jaws' position does not [MES-67] | CTRL |
| P2-R-16 | Perception across error above the slot's margin | 1 | 3 | 3 | Drops below 95 % with the pose, not with the config | The guide (capture −10/+6 mm [MES-63]); cut-off 2026-11-14: targets from `connect4_grid.yaml`, markers as a check. **Closed for the sim**: 80 drops of 80 in the asked column, aimed with the pose the camera gives [MES-71] | VIS |
| P2-R-17 | GitHub Actions minutes: about 2,500 of 3,000 this month (estimated, the billing API is out of reach). Out of minutes, no required check runs, and nothing merges | 2 | 3 | 6 | Settings → Billing | Check it now; the game job is separate; dispatch fewer nightlies | OPS |
| P2-R-18 | 8/10 is a noisy gate: at p = 0.90 it passes 93 % of the time (binomial) | 1 | 2 | 2 | A red night on a good commit | `release.sh` reads the newest report on the SHA: a second dispatch needs no new commit | QA |
| P2-R-19 | The robot's turn is tried again without a limit: with the dispenser's markers hidden, or a station the pick keeps missing, the console says `human_done` at each `WAIT_HUMAN` and the arm tries again every 6 to 15 s (review of 2026-11-13) | 1 | 2 | 2 | `lost` grows in `/connect4/state` | `n` or `s` in the console; a limit of 3 tries then `ERROR` is not built | CTRL |
| P2-R-20 | A pose of the grid or of the dispenser is published from one image with no quality gate: a marker half hidden may shift it by a few mm (the reprojection error is computed, not used) | 1 | 2 | 2 | A pick or a drop refused `not_edge_on` or `unsafe_path` with nothing moved | The brain judges each move on the pose (§2.5 of `P2-02`): a bad pose is refused, not played; a threshold as the calibration's `max_reprojection_px` is not built | VIS |
| P2-R-21 | A read of the stations is fresh by its reception time, not its image's: right after a fill, a read from an image taken before it may be taken (the reader lagging over 0.4 s) | 1 | 1 | 1 | A second `dispenser_empty` right after a fill | It heals at the next read; the reader's stamp on the stations is not built | VIS |
| P2-R-05 | Tokens jam or jitter in the sim slots | 1 | 2 | 2 | Jammed tokens in the spike | 378/378 drops on the working set [MES-63]. **Closed**: no token jammed in 80 drops of 12 games [MES-71], 150 more in [MES-73] | CTRL |
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
| Unit | Positions from perception (P2-02 §2.5): `clearance.py`, the distance of points to boxes in a frame, the arm's points from the URDF's meshes, a move judged along its joint lines; `drop.edge_on` and `drop.wait_pose`; the brain's refusals and `check_only` on a fake arm; `placement.py`, one check per column and station, asked again when a pose has moved | pytest | ADR-042 |
| Package | Each package with its own fakes: `so101_driver` (services), `so101_brain` (`/brain/drop`, fake driver), `so101_connect4` (fake camera, fake brain). colcon's coverage counts per package, and only the test's process: a bringup test covers no driver code, a launched node no line, so `test_drop_services.py` runs the brain in its own process | pytest, `colcon test` | Quality gate, 80 % on new code |
| Package | `so101_connect4` on its fake stack: `game_node` (`test_game_node.py`), the client and the console (`test_client.py`, `test_play.py`), the parameters of the two nodes, each with its range (`test_parameters.py`) | pytest, in-process nodes | P2-05, P2-08 |
| Package | `so101_bringup`: the game bench's score and CSV (`test_games.py`), the simulated person (`test_sim_hand.py`), the robustness cases' verdicts (`test_robustness.py`, `test_robustness_game.py`) | pytest | P2-06 |
| Integration (sim) | The game's whole stack as processes (`test_connect4_game_launch.py`): the QoS of the game's topics against `01` §3.1; a game cut by the bench's timeout fails as `bench_timeout`, keeps its rosbag, and the game node is told to stop. The robustness demo of the game on a fake stack (`test_robustness_game_stack.py`) | `so101_bringup/test`, launch_testing | P2-06, P2-08 |
| Integration (sim) | Scene: tick and image rate with 42 tokens; a render finds the 4 markers | `so101_bringup/test` | #79 |
| Integration (sim) | Positions from perception (P2-02 §2.5): the brain's model of the arm against MuJoCo's distance to the grid, within 3 mm; a game's moves keep twice the 5 mm margin; the grid 2 cm nearer or 5 mm further is refused; on the stack, the grid set down elsewhere between two games is found, judged and played | `so101_bringup/test/test_connect4_clearance.py`, `test_connect4_game_launch.py` [MES-78] | #260 |
| Integration (sim) | Board reader against the scene's truth (tests only, C-06), under the default light and the dim one: 10 states each in `colcon test`, 50 once as a MES [MES-68], [MES-73] | `so101_bringup/test` | P2-01 |
| Integration (sim) | One pick and drop in `colcon test`, the board reader running and the targets on its poses; 50 once as a MES, random columns; 10 tries without a token | `so101_bringup/test/test_connect4_drop_launch.py` (`CONNECT4_DROPS=50`) [MES-69], [MES-77], [MES-78] | P2-02, P2-03 |
| Integration (sim) | The dispenser: its pose from its own markers over 20 placements, the stations read, the pick at each corner of the token's play (±1.5 mm) and of the camera's error, the grid set down elsewhere with its tokens | `so101_bringup/test/test_connect4_dispenser.py` [MES-76], [MES-77] | P2-02, P2-03 |
| End-to-end (sim) | Game bench: N games against a scripted human, random or engine | `evaluate --games` (`08`) | P2-06 |
| End-to-end (sim) | Robustness demo of the game: the course's five fault questions asked of a game, a stop mid-move, and the grid set down elsewhere (played 2 cm away; refused where the arm would hit it), one command each | `tools/so101 robustness g1` to `g8` (#243, #260), `results/robustness_demo.md` | `06` §10, P2-03, P2-05 |
| Spike | 100 drops, 100 with ±5 mm TCP offset, capture sweep, full stacks | `experiences/connect4/scene/` | Go / no-go, #76 |
| Real (`v2.1.0`) | 30 drops, 30 handovers, 20 board photos, 3 filmed games; guide vs camera if the wrist camera is kept | manual protocol, results in `results/` | `P2-01` real parts |

### 3.1 Phase 2 in the CI

| When | What |
| --- | --- |
| Every PR | The 7 required checks, unchanged in name: lint, guards (extended to `so101_connect4`, plus no `/joint_command` in the game and no `/sim/` in the pipeline), both builds and tests, phase-1 E2E. New: `E2E sim (1 game)`, one game against a random player on the compose stack in game mode (`evaluate --games 1 --min-success 1`), **not required at first**, so a flaky game cannot freeze every PR in the last week; it joins the required checks on Thu 19 if every run was green (#233) |
| Each night `main` changed | The phase-1 bench, then 10 games against a random player on the stack started again in game mode (`down`, then `up`); one verdict line, `Target ≥ 90 % and ≥ 8/10 games: met.`, read by the report and by `release.sh`; SonarQube Cloud, coverage per package, trends |
| Release | `release.sh` needs the bench on the exact commit, the gate and the CHANGELOG (`15` §6). The gate is read **before** the last PR (#227) |

Ten games against a random person last about 20 min with the dispenser: 86 to 187 s a game [MES-77] (12 min and 58 to 126 s when the token was given by hand, 7 to 16 tokens [MES-71]). Against the engine as the person a game goes up to 40 tokens and 5 min, so the night plays the random one. The job has 120 min and uses 19 without the games. Not worth doing here: CI or Sonar badges, a Sonar scan on PRs, path filters, mypy, a matrix. Mutation tests (mutmut) run by hand, as phase 1's: `turns.py` 95.4 % and `drop.py` 90.1 % of the mutants killed, the pick included [MES-74], [MES-77] (`08` §1).

Who sees it: the repository is private and the school copy has no Actions, Issues or Releases, so the visible proof is in the tree: `.github/workflows/`, `results/phase2_2026-11-13.md`, the README (#234).

Every result enters the register of `../11_cross_review_and_measurements.md` as a MES, like phase 1.

## 4. Questions for the teaching staff

Answered on 2026-10-16, with the questions of `../README.md`. *Team* marks what the team settled itself.

1. **Riser** (*team*): the arm may be placed as the team wants, riser included; on the riser it goes through the `07` §3 commissioning again (`P2-02` §2.4).
2. **Cameras** (*team* and staff): a grid webcam and a wrist camera are both fine, lent by the school. Any USB webcam works for the grid; the wrist camera is probably the WowRobo 2 MP USB module, with a printable mount (STL on Teams); availability per campus: the staff will confirm (ADR-038 update).
3. **Real-arm sessions** (*team*): access whenever needed.
4. **Taught poses** (staff): reuse phase 1 as much as possible; the fallback is accepted if the PR and the defense explain why and show that the computed poses were tried and fixed first (`P2-02` §2.4).
5. **3D printer** (*team*): available on request.
