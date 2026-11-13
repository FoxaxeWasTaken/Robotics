# P2-01 — Scope, levels and demo scenario

> Authors: [CP] Project lead · Reviewers: [DEC], [QA], [SAFE]
> Requirements: C-53, C-54 · Decisions: ADR-034, ADR-037, ADR-038 · Roadmap: `P2-04`

## 1. Goal

A complete game of Connect Four between a human and the arm, the robot reading the grid by itself, choosing its move and playing it without help, filmed with the team on camera (C-53).

**`v2.0.0`, the hand-in of 2026-11-20 23:42, is the game in simulation.** The real arm is `v2.1.0`: phase 1 already runs on it, and the two weeks left go to a game that really works rather than to a riser and a handover (ADR-034 Update).

## 2. Levels

The plan is built so that a lower level is always demonstrable if a higher one slips.

| Level | Content | Shown at the defense as |
| --- | --- | --- |
| **0 — Fallback** | Tic-tac-toe with the phase-1 cubes on a 3×3 grid drawn on the table: no vertical grid, no slot, same software (board reader, engine, task layer) | Only if the drop spike (`P2-04` step P2.1) stays a no-go after changing the sim geometry |
| **1 — Target of `v2.0.0`** | Connect Four **in sim**: a person plays from a terminal (`tools/so101 play`), the robot reads the grid with its camera and plays; full games measured on the bench and in the CI | Video of the sim + RViz, the team playing on camera, bench numbers |
| **2 — Target of `v2.1.0`** | Connect Four **on the real arm** against a person: store-bought grid with its depth guide, the arm on a riser, tokens handed to the robot, webcam facing the grid, wrist camera if the drop bench needs it; the human presses "token given" and "done" | Live demo or video with the team, real numbers |
| **3 — Stretch** | Pick one or two: difficulty levels; automatic detection of the human's move (no key); the robot comments the game (LLM + text-to-speech, **never** in the motor loop); a token rack or dispenser instead of the handover; continuous visual servoing (Pinocchio, ADR-008 "reconsider if") | Bonus clip |

## 3. Demo scenario (what the video shows)

1. The empty grid in the sim, the robot waiting with its gripper open, the terminal of `tools/so101 play` next to it. The team is on camera, around the screen.
2. A player presses a column: a red token falls in the grid, a yellow token appears between the robot's jaws.
3. The robot looks (perception view in RViz: grid outline, 42 cells classified), thinks (chosen column shown), closes its jaws on the token, drops it into the column and comes back with its gripper open.
4. The game goes on until a win or a draw; the console announces the result.
5. Cut to the numbers: games completed, drop success rate, board reading accuracy, and the CI that plays a game on each PR.

On the real arm (`v2.1.0`), the same scenario with a person: the token handed between the jaws, "token given" then "done".

## 4. Measurable targets (team-defined)

`P2-xx` are **targets chosen by the team**, not imposed requirements (the only imposed requirement is C-54).

| ID | Target | Measured by |
| --- | --- | --- |
| P2-01 | Board reading: **100 %** of the cells right on 50 sim board states. *`v2.1.0`: ≥ 95 % of the boards fully right on 20 real photos* | `so101_bringup/test/test_connect4_reading.py`: 10 states in `colcon test`, 50 measured [MES-68] (sim ground truth allowed in tests, C-06) |
| P2-02 | Drop into the requested column: **≥ 95 %** in sim (N = 50). *`v2.1.0`: ≥ 90 % on the real arm (N = 30)* | **50/50** [MES-69]: `test_connect4_drop_launch.py` with `CONNECT4_DROPS=50`; one drop in `colcon test` |
| P2-03 | The token is held from the close to the release, **≥ 98 %** in sim; an empty gripper is **always** detected (the arm never plays without a token). *`v2.1.0`: ≥ 95 % on a real handover* | Held **50/50**, empty jaws refused **10/10**, the arm still [MES-69]; the same test, and `so101_brain`'s `test_drop_services.py` on a fake arm |
| P2-04 | Engine: only legal moves; wins 100/100 against a random player; move chosen in **< 2 s** | Unit tests |
| P2-05 | The arm **never moves** during the human's turn | `turns.py` tests; `test_game_node.py`: no call to the brain in `WAIT_HUMAN` or `FIX_BOARD`; the game bench fails a game where an arm joint moved 0.05 rad during a person's turn: none in 12 games [MES-71]. *`v2.1.0`: speeds near a person, `P2-02` §8* |
| P2-06 | **≥ 8/10** full games in sim without intervention. *`v2.1.0`: ≥ 3 full games filmed on the real arm* | **10/10** against a random person, 2/2 against the engine [MES-71]; the game bench, `evaluate --games`, one game on each PR and ten each night; videos |
| P2-07 | Anyone can play against the sim robot by following the README (public repository goal) | Replayed by a member who did not write the game, from a fresh clone (as MES-26) |
| P2-08 | Same quality rules as phase 1: pure modules with tests, CI green, guards C-05 / C-06 extended to the new package; a game on each PR, the game bench in the nightly | CI (`P2-03` §3.1) |

## 5. Out of scope

- In `v2.0.0`: the real arm, the riser, the depth guide as an object, the handover by a hand, a grid webcam, the wrist camera (`v2.1.0`, `P2-04`).
- A grid facing the robot: out of reach of a 5-DoF arm [MES-27] (ADR-035).
- Perfect play from an opening book or a solved-game database: a depth-limited search is enough to beat a casual player and keeps the code small.
- Any motor command issued by an LLM or by the game node directly: motions go through the brain only (ADR-037).
