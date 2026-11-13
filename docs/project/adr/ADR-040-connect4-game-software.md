# ADR-040 — Connect Four in sim: the engine, the turns, the reading of the grid, the bench

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (built and measured on the sim) | 2026-11-13 | DEC → VIS, KIN, QA, SAFE | C-05, C-06, C-42, C-54 |

## Context

ADR-034 to ADR-038 decide the application, the grid, the scene, the task layer and the hardware. The software of the game itself was decided step by step from 2026-11-07 to 2026-11-13, each choice on a measurement [MES-63 to MES-73], and written only in `phase2/P2-02`. This ADR records those decisions in one place, as phase 1 did for its own.

## Options considered

| Question | Kept | Rejected, and why |
| --- | --- | --- |
| The engine | Our negamax with alpha-beta on bitboards, pure Python, iterative deepening within 1 s | A solver library: a dependency for 200 lines, and nothing to explain at the defense. A perfect solver: a robot nobody beats is a poor demo |
| The levels | Three search depths measured in order: easy 1, medium 4, hard 10 | "Depth = strength": depth 2 does not lose to depth 6 (15 – 15) [MES-70] |
| Where the search runs | A `ProcessPoolExecutor` with the `forkserver` context | A thread: the GIL delays the node's 10 Hz tick, as the grasp planner's did [MES-40], [MES-66] |
| Whose turn it is | The person says it: `/connect4/human_done`; then the camera must show one more red token | Detecting the move from the images alone: a hand over the grid, a token in flight; cut on 2026-11-12 |
| A grid that is not the one expected | `FIX_BOARD`, with its reason: the arm stays still until a person puts it right; what the camera shows wins over what the engine planned | Guessing, or replaying the move: the arm would move on a board it does not understand |
| When the arm moves | In `MOVE` only, one `/brain/drop` at a time; the game node never publishes `/joint_command` (ADR-037) | A second commander of the arm |
| Finding the grid | Four ArUco markers on its frame, its pose from 3 or 4 of them, with our calibration of the phase-1 camera, unmoved, at 640×480 | A second camera facing the grid (ADR-036's `board_cam`): not needed, 2100 cells of 2100 [MES-65], [MES-68] |
| Reading a cell | The share of a disc, on the tokens' face, in a color's HSV range; a board is published once 5 images in a row agree | One image: a token in flight or a hand reaches the game [MES-66] |
| The images' QoS | The board reader subscribes Reliable, depth 1 | Best effort, as perception: 1 image of the 10 Hz at 640×480 [MES-67] |
| The robot's token | Given between its open jaws at `HANDOVER`; the brain judges the grip | A dispenser or a rack: `v2.1.0` at the earliest (ADR-038) |
| Where the arm rests | `HANDOVER`, also the driver's last pose in game mode | `PARK`: the joint line to it crosses the grid's near post [MES-69] |
| The speed of a move | The motion's speed with the token held and next to the guide; 1.4 times it on the two legs back, jaws empty | Faster with the token, or the lift as one motion: the clearance to the guide was measured on the stop-and-go lift [MES-63], [MES-73] |
| The proof | A game bench with a scripted person (random, or the engine), scored on the true board; one game on each PR, ten each night, one verdict with the pick & place's | A demo by hand only: phase 1's bench is what caught its regressions (ADR-018) |

## Decision

- **Engine** (`so101_connect4/game.py`): negamax, alpha-beta, center-first ordering, a transposition table, iterative deepening; the level is the depth, `engine.max_depth`, the only parameter that changes while the node runs; a seed decides between equal moves, so a game can be replayed.
- **Turns** (`turns.py`, pure): `IDLE`, `WAIT_HUMAN`, `READ_BOARD`, `THINK`, `MOVE`, `GAME_OVER`, `FIX_BOARD`, `ERROR`. A timeout on every state that waits for a machine; none on those that wait for a person. Only `MOVE` commands the arm.
- **Reading** (`board.py`, `board_reader_node`): the grid's pose from its markers, the 42 cells by color, a board checked (gravity, counts) and published once stable, valid or not, with its reason. No ground truth and no provided extrinsics (the guards cover the package).
- **Moves** (`so101_brain/drop.py`, ADR-037's Update): one `/brain/drop` from `HANDOVER`, every pose solved before anything moves; the speeds above.
- **Proof** (`so101_bringup/games.py`, `08` §3.1): `evaluate --games`; a game fails on `FIX_BOARD`, `ERROR`, a lost token, a token in another column, an arm joint that moved 0.05 rad during a person's turn, or the timeout. Target P2-06: 8 games of 10.
- **Sim only for `v2.0.0`**: the real arm's values (speeds near a person, a webcam's distortion, the handover's tolerance) come with `connect4_real.yaml` in `v2.1.0` (`phase2/P2-02` §8).

## Consequences

- The engine, the turns and the reading are pure modules with unit tests; the nodes only store messages and call them.
- A person can play from a terminal (`tools/so101 play`), and the bench plays through the same client: the two cannot drift.
- `turns.py` and `drop.py` join phase 1's mutation testing (mutmut, by hand, `08` §1): 95.9 % and 88.6 % of their mutants killed, the survivors equivalent or exact-threshold ties [MES-74].

## Evidence / validation

50 tokens of 50 in their column, held 50 of 50, empty jaws refused 10 of 10 [MES-69], [MES-73]; 2100 cells of 2100 on 50 game states, under the default light and a dim one [MES-68], [MES-73]; 10 games of 10 against a random person, 2 of 2 against the engine, no token lost in 80 drops, the arm still during each person's turn [MES-71]; the six fault cases of a game handled (`results/robustness_demo.md`).

## Reconsider if

- The real grid's reading falls under 95 % of the boards (P2-01): a webcam facing the grid (ADR-038).
- A person beats the hard level too easily at the defense: a better leaf evaluation, not a deeper search [MES-70].
