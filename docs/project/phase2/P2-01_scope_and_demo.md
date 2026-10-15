# P2-01 — Scope, levels and demo scenario

> Authors: [CP] Project lead · Reviewers: [DEC], [QA], [SAFE]
> Requirements: C-53, C-54 · Decisions: ADR-034, ADR-038 · Roadmap: `P2-04`

## 1. Goal

A complete game of Connect Four between a human and the arm, **in simulation first, then on the real arm**, filmed with the team on camera (C-53). The robot reads the grid by itself, chooses its move and plays it without help.

## 2. Levels

The plan is built so that a lower level is always demonstrable if a higher one slips.

| Level | Content | Shown at the defense as |
| --- | --- | --- |
| **0 — Fallback** | Tic-tac-toe with the phase-1 cubes on a 3×3 grid drawn on the table: no vertical grid, no slot, same software (board reader, engine, task layer) | Only if the drop spike (`P2-04` step P2.1) is a no-go |
| **1 — Minimum** | Connect Four **in sim**: the "human" plays through a sim service, the robot reads the grid with its camera and plays; full games measured on the bench | Video of the sim + RViz, bench numbers |
| **2 — Target** | Connect Four **on the real arm** against a person: store-bought grid with its depth guide, the arm on a riser, tokens handed to the robot, webcam facing the grid, wrist camera if the drop bench needs it; the human presses "token given" and "done" | Live demo or video with the team, real numbers |
| **3 — Stretch** | Pick one or two: difficulty levels; automatic detection of the human's move (no key); the robot comments the game (LLM + text-to-speech, **never** in the motor loop); a token rack or dispenser instead of the handover; continuous visual servoing (Pinocchio, ADR-008 "reconsider if") | Bonus clip |

## 3. Demo scenario (what the video shows)

1. The empty grid, the robot waiting with its gripper open, the human facing the grid.
2. The human drops a red token, puts a yellow token between the robot's jaws ("token given": the gripper closes) and presses "done" (level 2); in sim, the same through the sim services (level 1).
3. The robot looks (perception overlay in RViz: grid outline, 42 cells classified), thinks (chosen column shown), drops its token into the column and comes back with its gripper open.
4. The game goes on until a win or a draw; the robot announces the result.
5. Cut to the numbers: drop success rate, board reading accuracy, games completed.

## 4. Measurable targets (team-defined)

`P2-xx` are **targets chosen by the team**, not imposed requirements (the only imposed requirement is C-54).

| ID | Target | Measured by |
| --- | --- | --- |
| P2-01 | Board reading: **100 %** of the cells right on 50 sim board states; **≥ 95 %** of the boards fully right on 20 real photos | `test_board_reader` (sim ground truth allowed in tests, C-06); real photo set |
| P2-02 | Drop into the requested column: **≥ 95 %** in sim (N = 100), **≥ 90 %** on the real arm (N = 30) | Drop bench (`P2-03` §3) |
| P2-03 | Handover: the token is held from "token given" to the release, **≥ 98 %** sim, **≥ 95 %** real; an empty gripper is **always** detected (the arm never plays without a token) | Handover bench (`P2-03` §3), 10 % of the trials without a token |
| P2-04 | Engine: only legal moves; wins 100/100 against a random player; move chosen in **< 2 s** | Unit tests |
| P2-05 | The arm **never moves** during the human's turn; speeds within the phase-1 real-arm limits (ADR-023) | Game state machine tests; review of the real sessions |
| P2-06 | **≥ 8/10** full games in sim without human intervention; **≥ 3** full games filmed on the real arm | Game bench; videos |
| P2-07 | Anyone can play against the sim robot by following the README (public repository goal) | Replayed by a newcomer from a fresh clone (as MES-26 for phase 1) |
| P2-08 | Same quality rules as phase 1: pure modules with tests, CI green, guards C-05 / C-06 extended to the new package | CI |

## 5. Out of scope

- A grid facing the robot: out of reach of a 5-DoF arm [MES-27] (ADR-035).
- Perfect play from an opening book or a solved-game database: a depth-limited search is enough to beat a casual player and keeps the code small.
- Any motor command issued by an LLM or by the game node directly: motions go through the brain only (ADR-037).
