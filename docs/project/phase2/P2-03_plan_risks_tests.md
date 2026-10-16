# P2-03 — Risks, tests and questions

> Authors: [CP] Project lead, [QA] Quality · Reviewers: all experts
> Requirements: C-54 · Decisions: ADR-034 to ADR-038 · Roadmap: [`P2-04`](P2-04_roadmap.md) (stage 7 of `09`)

## 1. Plan

Moved to its own roadmap: [`P2-04_roadmap.md`](P2-04_roadmap.md) (steps P2.0 to P2.9, same rules as `09`).

## 2. Risk register

Same scale as `10` (P = probability, I = impact, 1 to 3; Crit. = P × I).

| ID | Risk | P | I | Crit. | Warning sign | Mitigation | Owner |
| --- | --- | --- | --- | --- | --- | --- | --- |
| P2-R-01 | Real drops miss the slot (accuracy, backlash, calibration) | 3 | 3 | **9** | Real drop bench < 90 % | Depth guide; the round token self-centers along the grid; wrist-camera look and correct (P2.8); calibration-zero check [MES-30]; slow straight descent; taught poses as a fallback | KIN, CTRL |
| P2-R-02 | Board misread (lighting, small cells, arm in the image) | 2 | 3 | 6 | Frequent `FIX_BOARD` | Camera facing the grid, ≥ 10 px per cell [MES-31]; manual exposure; read only with the arm out of view (`HANDOVER`); validation by the rules | VIS |
| P2-R-03 | Calibration zero ≠ URDF zero on the real arm | 2 | 3 | 6 | Rest-pose TCP measured with a ruler ≠ FK | Measure first; per-joint offset or recalibration (`07` §4) | CTRL |
| P2-R-04 | Phase 2 eats phase-1 time (both graded at S7) | 2 | 3 | 6 | Phase-1 bench below its target | Start after the vertical slice; level ladder; stretch only at the end | CP |
| P2-R-05 | Tokens jam or jitter in the sim slots | 2 | 2 | 4 | Jammed tokens in the spike | Tune `margin`, `solref`, slot clearance, token mass | CTRL |
| P2-R-06 | Token badly placed at the handover (off-center along the grid, not held, slips during the move) | 2 | 2 | 4 | Drops off by one column, tokens lost | Held-token check by the gripper position; handover offsets in the spike; printed jaw pads with a recess that centers the token; wrist-camera check (P2.8) | CTRL |
| P2-R-07 | Hand in the workspace while the arm moves, or fingers pinched at the handover | 1 | 3 | 3 | — | Arm still while waiting; the gripper closes only on "token given", slowly, to a squeeze target just below the token thickness; the arm moves only after `/connect4/human_done`; `/brain/stop` at hand; two people at real sessions (`07` §7) | SAFE |
| P2-R-08 | `PickPlace` interface change late or rejected | 1 | 2 | 2 | — | ADR-037 reviewed early; brain refactored in small PRs behind the phase-1 bench | ARC |
| P2-R-09 | Python engine too slow for a useful depth | 1 | 1 | 1 | Depth < 5 in 1 s | Bitboards, move ordering, transposition table; cap the depth | DEC |
| P2-R-10 | The purchased grid does not fit the arm (other dimensions than the model) | 1 | 2 | 2 | P2.1 kinematics rerun | Riser height and grid radius from the measurements; travel-size grid or custom grid (ADR-035) as fallbacks | KIN |
| P2-R-11 | Wrist-camera cable snags, or the mount shifts | 2 | 1 | 2 | Image shifted in the known check pose | Cable slack sized for a ±90° wrist roll; check pose at every session | CTRL |

## 3. Tests and measurements

| Level | What | Tool | Target |
| --- | --- | --- | --- |
| Unit | `game.py`: wins in the 4 directions, draw, full column, known positions, 100 games against random, time per depth | pytest | P2-04 |
| Unit | `board.py`: synthetic boards (random states, noise, brightness changes) | pytest | P2-01 |
| Unit | `drop_ik`: jaw error < 4° on the 7 columns of the chosen layout; a few MES-27 points as regression | pytest (kinematics module, `04` §9) | ADR-035 |
| Integration (sim) | Drop bench: 100 drops, random column, ±5 mm perturbation; handover bench: 100 `give_token` with random offsets, 10 of them without a token | launch_testing | P2-02, P2-03 |
| Integration (sim) | Drop bench with ±10 mm perturbation, with and without wrist-camera look and correct | launch_testing | P2.8 |
| Integration (sim) | Board reader against the scene's ground truth (tests only, C-06) on 50 states | launch_testing | P2-01 |
| End-to-end (sim) | Game bench: 10 games against a scripted random human | evaluation bench (`08`) | P2-06 |
| Real | 30 drops, 30 handovers, 20 board photos, 3 filmed games | manual protocol, results in `results/` | P2-01 to P2-03, P2-06 |
| Real | Guide vs camera: 30 drops each with guide only, camera only, guide + camera | manual protocol, results in `results/` | P2.8: the simplest configuration ≥ 90 % is kept |

Every result enters the register of `../11_cross_review_and_measurements.md` as a MES, like phase 1.

## 4. Questions for the teaching staff

Answered on 2026-10-16, with the questions of `../README.md`. *Team* marks what the team settled itself.

1. **Riser** (*team*): the arm may be placed as the team wants, riser included; on the riser it goes through the `07` §3 commissioning again (`P2-02` §2.4).
2. **Cameras** (*team* and staff): a grid webcam and a wrist camera are both fine, lent by the school. Any USB webcam works for the grid; the wrist camera is probably the WowRobo 2 MP USB module, with a printable mount (STL on Teams); availability per campus: the staff will confirm (ADR-038 update).
3. **Real-arm sessions** (*team*): access whenever needed.
4. **Taught poses** (staff): reuse phase 1 as much as possible; the fallback is accepted if the PR and the defense explain why and show that the computed poses were tried and fixed first (`P2-02` §2.4).
5. **3D printer** (*team*): available on request.
