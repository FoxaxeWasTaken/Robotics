# ADR-037 — Phase 2: a task layer driving the brain through a `PickPlace` action

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| RECOMMENDED (interface change: approval of the brain and perception owners in the implementing PR, `01` §3) | 2026-10-09 | ARC → KIN, DEC, VIS | C-05, C-33, C-54 |

## Context

The game must turn "play column c" into motions. The brain already owns kinematics, trajectories, grasp detection and the safety-related motion rules (`04`, `06`, ADR-023); phase 2 must reuse them (C-54) without duplicating them.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| The game node solves IK and publishes `/joint_command` | No brain change | Duplicates kinematics and safety rules; two writers on `/joint_command` |
| A game-specific brain service ("drop in column c") | Small | The brain learns the game; nothing reusable |
| **A generic `PickPlace` action served by the brain** | Reuses the phase-1 motion code; long-running with feedback and cancel (the ROS 2 idiom); phase 1's `/brain/start` becomes a client of the same code | New interface; the phase-1 brain is refactored behind it |

## Decision

- New package **`so101_connect4`**: `game.py` and `board.py` (pure, tested), `board_reader_node` (perception container), `game_node` (control container).
- The brain serves **`/brain/pick_place`** (`so101_interfaces/action/PickPlace`). Goal: pick pose, place pose, orientation constraint (free, point down, or jaw axis normal to a given vector), release height. Feedback: current phase. Result: success and message.
- The game node never publishes `/joint_command`.
- The interfaces of `phase2/P2-02` §6 become contract in `01` §3 when the implementing PR is approved by the owners on both sides.

## Consequences

- One motion code path for phase 1 and phase 2: the phase-1 bench guards against regressions.
- The C-05 and C-06 guards (`tools/ci/guards.sh`) are extended to `so101_connect4`.

## Evidence / validation

Design review (this ADR). To validate: the phase-1 bench unchanged after the refactor, and 50 sim drops through the action (`phase2/P2-04` step P2.5).

## Reconsider if

The refactor threatens the phase-1 schedule: then a game-specific service first, the generic action later.
