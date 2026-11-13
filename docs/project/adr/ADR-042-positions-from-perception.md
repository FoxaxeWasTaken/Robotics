# ADR-042 — Connect Four: the brain judges each move where the camera sees the grid and the dispenser

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (built and measured on the sim, #260) | 2026-11-13 | CTRL → KIN, VIS, SAFE | C-05, C-06, C-54 |

## Context

The team's rule of 2026-11-13: as much as possible goes through perception, nothing about where things stand is given to the arm. The game already aims each move at the grid's and the dispenser's measured poses (ADR-040, ADR-041). Two things were still given:

- the joint pose `HANDOVER` the arm waits at, chosen for the layout's dispenser;
- the legs of a move: a fixed sequence, measured clear of the grid where the layout puts it [MES-73], and not judged against the grid where it really stands.

A grid set down elsewhere was aimed at correctly, and could be hit on the way. How near is measured in [MES-78]: 11 mm where the layout puts the grid, 2 mm from the shoulder with the grid 2 cm nearer, 3.5 mm from the lower arm with it 5 mm further. Every column still has a pose in both cases, so the IK refuses nothing. The scene draws the grid's posts and base without contacts: no game showed it.

A second limit is the arm's own. It has 5 axes and holds a token in its vertical plane (ADR-035): a slot turned out of that plane gets a token turned by as much, and the token falls beside its column [MES-79].

## Options considered

| Option | For | Against |
| --- | --- | --- |
| A tolerance box around the layout's placement, checked by the game node | A few lines | It keeps the layout's placement in the nodes, which the rule forbids; the box is not one: the limits depend on the column and on the leg |
| The arm as its polyline with a radius per segment (as `kinematics.self_collides`) | Pure numpy, cheap | Measured: the upper arm's hull is up to 54 mm from its line, the tool's 36 mm, and a game's moves pass 11 mm from the grid. Radii large enough to be safe refuse every move |
| Pinocchio with hpp-fcl on the URDF's collision meshes | Exact distances, a maintained library | A new runtime dependency of the brain one week before the hand-in; the arm image has it, the code never used it |
| **Points on the convex hulls of the URDF's collision meshes, against boxes** | The hulls MuJoCo collides with; numpy and scipy, already there; within 1.4 mm of MuJoCo on 99 % of 68,000 poses, 3.6 mm at most [MES-78] | 2 s to load the meshes at the brain's start; the volume to avoid must be boxes |
| A planner that finds another way | Would play on more placements | Out of scope (#260): the legs stay the measured sequence, judged |

Who judges: the game node knows the layout, the brain knows the arm. Reach, the arm's plane and the arm's volume are the brain's (ADR-037: every motion is the brain's), so the brain judges and the game asks.

## Decision

- **The brain subscribes to `/connect4/grid_pose` and `/connect4/dispenser_pose`.** Without the pose a move needs, it refuses: `no_grid_pose`, `no_dispenser_pose`.
- **A drop and a pick are refused when their slot is not edge-on** (`not_edge_on`): the angle between the slot's plane, as measured, and the arm's at the release, within −6° to +1° for a drop and ±4° for a pick [MES-79].
- **A move is refused when a leg passes nearer than 5 mm of the grid** (`unsafe_path:<step>`). The arm is a cloud of points every 8 mm on the convex hulls of the URDF's collision meshes, the moving jaw at the opening the steps command; the grid is two boxes in its measured frame, its body and its depth guide, from its own dimensions (`connect4.grid_boxes`). Each step's joint line is judged every 0.03 rad (`so101_brain/clearance.py`). It holds for `/brain/drop`, `/brain/pick` and `/brain/handover`.
- **The arm waits beside the dispenser where the camera sees it** (`drop.wait_pose`): the tool's place and tilt in the dispenser's frame are constants, the joints are solved at each `/brain/handover`. `connect4.handover_pose` stays: the pose while no dispenser has been seen (`token_source: hand`), then the IK's seed.
- **`Drop.srv` gets `check_only`**: the same judgment, replied at once, nothing moves. Interface change (ADR-037).
- **The game says `ready` once the brain would play every column and take a token at every station** (`so101_connect4/placement.py`), and asks again when a pose has moved by 5 mm or 2°. `/connect4/state` gets `not_ready`, the reason.
- **The sim can set the grid down elsewhere** (`/sim/connect4/move_grid`, `MoveGrid.srv`): the simulated person writes the grid's pose in the model. A mocap body, moved through `MjData`, kept the tokens resting in the grid awake: they crept and the tick slowed [MES-78].

Not decided here: the dispenser as an obstacle (the arm's ways beside it are fixed relative to it, and measured [MES-77]); a tilted grid (the camera measures a tilt, nothing uses it); the real arm (`v2.1.0`: the same judgment, on the purchased grid's dimensions).

## Evidence

- [MES-78] (`experiences/connect4/drop/e13_grid_clearance.py`, `so101_bringup/test/test_connect4_clearance.py`): the model against MuJoCo's distance; how near a game's moves pass, on the layout's grid and on grids set down elsewhere.
- [MES-79] (`experiences/connect4/drop/e14_edge_on.py`): drops and picks with their contacts, the grid turned about a column and the dispenser about a station, −8° to +8°.
- On the stack: `test_connect4_game_launch.py` (the grid moved between two games, found, judged and played), `tools/so101 robustness g7` and `g8` (`results/robustness_demo.md`).

## Reopen if

A placement the team needs is refused though the arm could play there by another way: then a planner, or a second sequence of legs. The purchased grid is not two boxes (#75). The measured pose is too noisy for a 5 mm margin on the real camera (#89): a larger margin, or a filtered pose.
