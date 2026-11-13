# Robustness demo in sim (step 5.4a, #103), 2026-10-27

The five robustness questions of the day-2 slides [BOOT2 p12], asked again by the teaching staff on 2026-10-16: for each one, a reproducible run on the sim stack, the expected behavior of `docs/project/06` §10, what the brain did, and whether it is **handled**, **partly** handled or **not handled**. The faults go through the sim-only services or the demo's own stack (its perception node stopped); the pipeline still sees the scene through the camera only, no ground truth (C-06).

## How to run a case

The demo starts its own stack (driver, brain and perception in one container, `so101_bringup/launch/robustness_demo.launch.py`), runs the case (`so101_bringup/robustness_demo.py`, cases and verdicts in `so101_bringup/robustness.py`), prints each `/brain_state` with the brain's reason for each transition (from `/rosout`), then the verdict. From the repository root, the workspace built (`tools/so101 up` once):

```bash
tools/so101 down                             # the demo runs alone: next to the everyday stack, /joint_states once stalled for 0.5 s
tools/so101 robustness 1                     # a new control container on ROS domain 78
tools/so101 robustness 5 --lighting dim -n 5 # case 5 under the dim light, 5 cycles (the default)
```

The case is `1`, `2`, `3a`, `3b`, `4` or `5`; `--lighting` and `-n` go to the launch's `lighting:=` and `episodes:=`. `tools/so101` refuses to start the demo while the everyday stack runs, and `--dry-run` prints the `docker compose run` line it would run. Each run takes 30 s to 2 min. To film a case, start RViz on the same domain first, in a second terminal (the display setup of `docs/project/02` §7):

```bash
docker compose -f docker/docker-compose.yml run --rm --no-deps -e ROS_DOMAIN_ID=78 -e DISPLAY control \
  bash -lc "source /ros2_ws/install/setup.bash && rviz2 -d /ros2_ws/install/so101_bringup/share/so101_bringup/rviz/so101.rviz"
```

## Results

Run on `feat/103-robustness-demo` (from `main` at `ebf5d87`), Windows 11 + WSL2 + Docker Desktop, driver seed 0.

| Case [BOOT2 p12] | Fault injected | Expected (`06` §10) | Observed `/brain_state` (brain's reason) | Verdict |
| --- | --- | --- | --- | --- |
| **1. The object moves mid-action** | `/sim/randomize` on entering `APPROACH` | Empty grasp, `RETRY`, a new lock, replan | `APPROACH` → `DESCEND` → `GRASP` → `RETRY` (*empty gripper: effort 0.000*) → `HOME_RETRY` → `WAIT_TARGET` (a new lock on the moved cube) → … → `DONE` (*cube seen in the box*), 27.6 s | **Handled** |
| **2. The grasp fails** | `/sim/randomize` on entering each `DESCEND` | Three empty grasps, then `ERROR:grasp_failed` | Three times `GRASP` → `RETRY` (*effort 0.000*) → `HOME_RETRY` → `WAIT_TARGET`; then `ERROR:grasp_failed` (*3 failed attempts*), 38.1 s | **Handled** |
| **3a. The object leaves the view** (during the transport) | The perception node stopped on entering `TRANSPORT` | The cycle completes on the last known values (C-33) | `TRANSPORT` → `RELEASE` → `HOME` → `VERIFY` → `DONE` (*warning: cube not seen for 2 s, assumed in the box*) | **Handled** |
| **3b. The object leaves the view** (before the cycle) | The perception node stopped before `/brain/start` | No target, no motion beyond `PARK` | `WAIT_TARGET` → `ERROR:object_lost` (*no target locked 10 s after PARK*), no `APPROACH` | **Handled** |
| **4. The target is out of reach** | Perception stopped, a fake cube published at r = 0.5 m (as `test_9` of `so101_brain`) | `ERROR:unreachable`, no command beyond `PARK` | `WAIT_TARGET` → `PLAN` → `ERROR:unreachable` (*no plan*, 1.5 s in `PLAN`), no `APPROACH` | **Handled** |
| **5. The lighting changes** | `lighting:=dim`: every light ×0.4, tinted (1.0, 0.8, 0.55), a darker, warmer room | The color thresholds still hold | **0/5** cycles: each one `ERROR:object_lost`; while the brain waits for its target, the cube is detected on every image, **the box on none** (below) | **Not handled** on this date; **handled** since #175: 5/5 cycles in each of 2 runs (MES-52, below) |

The table gives the last run of each case, with the delivered code. Cases 1 to 4 behave as `06` §10 says in every run (case 1: 8 runs, case 4: 7, cases 2, 3a and 3b: 4 each), except one case-1 run stopped by the `/joint_states` stall described below, before the fault mattered.

### Case 5: the lighting

| Lighting | Cycles in `DONE` | Detections while the brain waits for its target, per cycle |
| --- | --- | --- |
| Default | 5/5 in 4 runs, 4/5 in 1 (the one failure: the `/joint_states` stall below, not the light) | cube and box on every image (9–41 images) |
| `dim` | **0/5**, in each of the 6 runs | cube on 99–100 % of the images, **box on 0** (86–136 images) |
| `dim`, since #175 | **5/5**, in each of the 2 runs | cube and box on every image (9–13 images) |

**Why** (MES-49, `docs/project/experiences/exp_lighting_hsv.py`, and the debug image of the clip), two causes that add up:

1. **The floor turns box-colored.** The warm light makes the grey checker (H 120, S 29) orange (H 14, S 99): **42 %** of the floor falls inside the box's HSV range. The box mask grows from 1,125 px (1,121 on the box) to **30,587 px, 29,864 of them on the floor**; in the debug image the light checker squares are drawn as box. Perception's shape guards reject such a blob.
2. **The box gets too dark.** Its brightness drops to V median **29**, below `hsv_box_v_min` = 40: only **11 %** of its pixels stay in range (99 % before).

The cube keeps V median 68 against its `hsv_cube_v_min` = 60 (81 % of its pixels in range): it survives, with little margin. The brain locks its target only with both the cube and the box, so it waits 10 s and stops with `ERROR:object_lost`, without moving: the failure is safe, but the task is not done.

Not fixed here (out of scope of #103: a weakness found by the demo becomes its own ticket). Brightness no longer separates the box from the floor (box V 25–50, floor V 26–43); saturation still does (box S 198–207, floor S 93–106). Leads for that ticket: a higher `hsv_box_s_min` with a lower `hsv_box_v_min`, a brightness-normalized threshold, or the camera's exposure and white balance on the real arm (`07` §4, step 6.4b).

**Fixed by #175** (MES-52): saturation floors between the lit floor and arm (S ≤ 113) and the box (S ≥ 164) and cube (S ≥ 205), V floors under their shaded sides (box S ≥ 140, V ≥ 20; cube S ≥ 150, V ≥ 30). Case 5 rerun on `fix/175-dim-light-box` (from `main` at `cdbd1d5`), the same machine, its own container: **5/5 cycles in `DONE` in each of 2 runs** under `dim`, cube and box on every image while the brain waits (9–13 images); the default-light bench 50/50.

## The same questions asked of the Connect Four game (#243), 2026-11-13

`tools/so101 robustness g1` to `g8`, the everyday stack stopped, as above: the demo's stack runs in game mode (`connect4:=true`, the board reader as its perception). Each case starts a game and waits for the person's first turn, then runs its steps (`so101_bringup/robustness.py`, `GAME_CASES`): an action, the `/connect4/state` or the reply that must follow, and whether the arm may move meanwhile (no arm joint more than 0.05 rad from its place at the step's start, target P2-05; after a refusal, which replies at once, the arm is watched one second more). The faults go through the simulated person's hand, the game's and the brain's services, or the demo's own board reader; never through the ground truth. One run each, in a throwaway container on the development machine, each verdict "handled"; the six were run again, with the same verdicts, once the arm's place was taken from the joints seen before a step's action and not after it (#252, found by a test of the demo on a fake stack), a third time, six of six, with the robot taking its tokens at the dispenser (#259; g2 and g6 below are that run's, a robot's move there is a pick then the drop, 20 s), and a fourth time with the positions from perception (#260): the six again and g7, g8, eight of eight. In that run g3 first ended `not handled`, `ERROR:brain_refused:no_joint_states` in the robot's pick, while the test suites and a mutation run loaded the machine: the brain's tick ran at 40 ms (p99) instead of 20.6 and the joints came late; run alone, handled:

| Case | Fault injected | Observed `/connect4/state` | Verdict |
| --- | --- | --- | --- |
| **g1. The scene changes mid-action** | The person drops two tokens in one turn | `READ_BOARD` → `FIX_BOARD:counts:2-0` in 0.7 s, the arm still; `new_game` clears the grid: `MOVE` → `READ_BOARD` → `WAIT_HUMAN` | **Handled** |
| **g2. The grasp fails** | The token dispenser emptied before the robot's move (`/sim/connect4/empty_dispenser`: 21 tokens laid on the table) | `THINK` → `MOVE` → `WAIT_HUMAN:dispenser_empty` 0.1 s into the move, on the camera's read of the stations: the arm still, no token counted lost; the dispenser filled and `human_done`: `THINK` → `MOVE` (20.3 s, a pick then the drop) → `WAIT_HUMAN`, the game goes on | **Handled** |
| **g3. The object leaves the view** | The board reader stopped, then the person plays | `READ_BOARD` → `FIX_BOARD:not_seen:human` at the 5 s timeout, the arm still; the reader started again publishes in 2.2 s, `human_done`: `THINK` → `MOVE` → `WAIT_HUMAN`, the game goes on | **Handled** |
| **g4. The target is out of reach** | `/brain/drop` at (0.6, 0, 0.3) m; a seventh token for a full column | The brain replies `unreachable` in 0.05 s, the driver `column 0 is full`; no state changes, the arm still | **Handled** |
| **g5. The lighting changes** (`--lighting dim`) | Every light ×0.4, tinted, as case 5 | A whole game against a random person: `GAME_OVER`, no token lost, none in another column | **Handled** |
| **g6. A stop in the middle of a move** (ours) | `/connect4/stop` 5 s into the robot's move | `ERROR:stopped` in 0.1 s, the brain `STOPPED` then `IDLE`, the arm frozen for the 2 s watched: at the dispenser, in its pick; `new_game`: `MOVE` (3.5 s: straight up out of the station, then back to `HANDOVER`) → `WAIT_HUMAN` | **Handled** |
| **g7. The scene changes between two games** (ours, #260) | After a turn, the person empties the grid and sets it down 10 mm nearer the arm and 3° around it, its far end 2 cm from where it was (`/sim/connect4/move_grid`). Nothing tells the nodes | `ready` false for 4.4 s, `not_ready: checking where the grid stands`: the brain is asked again whether it would play the 7 columns and take a token at the 3 stations, on the pose the camera now measures; `new_game`: `MOVE` → `WAIT_HUMAN`; a turn: `THINK` → `MOVE` (20.3 s) → `WAIT_HUMAN`, the robot's token read in the grid where it now stands | **Handled** |
| **g8. The target is out of reach** (ours, #260) | The grid set down 25 mm nearer the arm: every column still has a pose, and the shoulder would pass through the grid's near post | `not_ready: column 1: unsafe_path:column` 3.8 s after the move (the brain's log: `-2.5 mm from the grid`); `new_game` is refused with that reason; `/brain/drop` asked directly replies `unsafe_path:column` in 0.2 s; the arm still | **Handled** |

One marker hidden still reads the 42 cells, two give `markers:2`: `so101_connect4/test/test_board_reader.py`, on images rendered from the game's scene.

## Limits and observations

- **3a, `VERIFY` with perception dead**: with no detection for 2 s, `VERIFY` ends in `DONE` with a warning, as `06` §3.1 specifies. A cube that left the table and a perception that stopped look the same; known and accepted.
- **3b stops perception** rather than hiding the cube from the camera: to the brain it is the same (nothing published), and moving the cube out of view would need a new sim service in the driver (out of scope).
- **Cases 1 and 2**: `/sim/randomize` moves the box too; the brain locks both again after `RETRY`.
- **A `/joint_states` stall on this machine**: twice in about 80 cycles, the brain saw no `/joint_states` for 0.5 s (two stacks sharing the CPU) or 1.36 s (the demo stack alone), and stopped with `ERROR:no_joint_states`, the arm frozen. The driver logged nothing. The nightly bench (Linux runner) has 50/50, so it may be specific to Windows + WSL2 + Docker Desktop (see the WSL2 clock steps of MES-37); not investigated here. Since then the driver names its late ticks (#176); the stall did not come back in the runs of #176 and #178 (MES-51), and #178 was closed as not reproduced.
- **Perception does not always stop cleanly on SIGINT**: in one of the 15 runs that stop it (the last case-4 run), the node died with `RuntimeError: Unable to convert call argument '0'` in `take_message`, exit code 1, instead of a clean stop. The demo's outcome does not depend on it (perception is meant to stop there). It is the rclpy SIGINT race of AGENTS.md, which the driver handles since #158 and the perception node since #177 (MES-50).

## Clips

One clip per case, recorded on 2026-10-27: RViz (the arm, the external camera and the perception debug image) next to the demo's terminal (the command, the `/brain_state` sequence and the brain's reasons). Kept out of Git (the 1 MB rule), on the pre-release [`robustness-demo-2026-10-27`](https://github.com/FoxaxeWasTaken/Robotics/releases/tag/robustness-demo-2026-10-27).

| Case | Clip | What it shows |
| --- | --- | --- |
| 1 | [`case_1_object_moved.mp4`](https://github.com/FoxaxeWasTaken/Robotics/releases/download/robustness-demo-2026-10-27/case_1_object_moved.mp4) (68 s) | `GRASP → RETRY (empty gripper: effort 0.000)`, *attempt 1/3 failed*, a new lock, `DONE` |
| 2 | [`case_2_grasp_fails.mp4`](https://github.com/FoxaxeWasTaken/Robotics/releases/download/robustness-demo-2026-10-27/case_2_grasp_fails.mp4) (110 s) | `>>> randomize` at each `DESCEND`, *attempt 3/3 failed*, `ERROR:grasp_failed` |
| 3a | [`case_3a_perception_lost_in_transport.mp4`](https://github.com/FoxaxeWasTaken/Robotics/releases/download/robustness-demo-2026-10-27/case_3a_perception_lost_in_transport.mp4) (65 s) | Perception stopped in `TRANSPORT`, `VERIFY → DONE` with the warning |
| 3b | [`case_3b_perception_lost_before_cycle.mp4`](https://github.com/FoxaxeWasTaken/Robotics/releases/download/robustness-demo-2026-10-27/case_3b_perception_lost_before_cycle.mp4) (56 s) | `WAIT_TARGET → ERROR:object_lost`, the arm in PARK |
| 4 | [`case_4_out_of_reach.mp4`](https://github.com/FoxaxeWasTaken/Robotics/releases/download/robustness-demo-2026-10-27/case_4_out_of_reach.mp4) (47 s) | The fake cube locked at (0.5, 0, 0.025), `ERROR:unreachable`, the arm in PARK |
| 5 | [`case_5_dim_lighting.mp4`](https://github.com/FoxaxeWasTaken/Robotics/releases/download/robustness-demo-2026-10-27/case_5_dim_lighting.mp4) (50 s) | `lighting:=dim`: the debug image draws the floor's light squares as box, no target locked, the arm stays in PARK (the clip ends during the first cycle, before its `ERROR:object_lost`) |
