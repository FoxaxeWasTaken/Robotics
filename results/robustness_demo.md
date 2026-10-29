# Robustness demo in sim (step 5.4a, #103), 2026-10-27

The five robustness questions of the day-2 slides [BOOT2 p12], asked again by the teaching staff on 2026-10-16: for each one, a reproducible run on the sim stack, the expected behavior of `docs/project/06` §10, what the brain did, and whether it is **handled**, **partly** handled or **not handled**. The faults go through the sim-only services or the demo's own stack (its perception node stopped); the pipeline still sees the scene through the camera only, no ground truth (C-06).

## How to run a case

The demo starts its own stack (driver, brain and perception in one container, `so101_bringup/launch/robustness_demo.launch.py`), runs the case (`so101_bringup/robustness_demo.py`, cases and verdicts in `so101_bringup/robustness.py`), prints each `/brain_state` with the brain's reason for each transition (from `/rosout`), then the verdict. From the repository root, the workspace built (`tools/so101 up` once):

```bash
tools/so101 down   # the demo runs alone: next to the everyday stack, /joint_states once stalled for 0.5 s
docker compose -f docker/docker-compose.yml run --rm --no-deps -e ROS_DOMAIN_ID=78 control \
  bash -lc "source /ros2_ws/install/setup.bash && ros2 launch so101_bringup robustness_demo.launch.py case:=1"
```

`case:=` takes `1`, `2`, `3a`, `3b`, `4` or `5`; case 5 adds `lighting:=dim` (and `episodes:=N`, 5 by default). Each run takes 30 s to 2 min. To film a case, start RViz on the same domain first, in a second terminal (the display setup of `docs/project/02` §7):

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
| **5. The lighting changes** | `lighting:=dim`: every light ×0.4, tinted (1.0, 0.8, 0.55), a darker, warmer room | The color thresholds still hold | **0/5** cycles: each one `ERROR:object_lost`; while the brain waits for its target, the cube is detected on every image, **the box on none** (below) | **Not handled** |

The table gives the last run of each case, with the delivered code. Cases 1 to 4 behave as `06` §10 says in every run (case 1: 8 runs, case 4: 7, cases 2, 3a and 3b: 4 each), except one case-1 run stopped by the `/joint_states` stall described below, before the fault mattered.

### Case 5: the lighting

| Lighting | Cycles in `DONE` | Detections while the brain waits for its target, per cycle |
| --- | --- | --- |
| Default | 5/5 in 4 runs, 4/5 in 1 (the one failure: the `/joint_states` stall below, not the light) | cube and box on every image (9–41 images) |
| `dim` | **0/5**, in each of the 6 runs | cube on 99–100 % of the images, **box on 0** (86–136 images) |

**Why** (MES-49, `docs/project/experiences/exp_lighting_hsv.py`, and the debug image of the clip), two causes that add up:

1. **The floor turns box-colored.** The warm light makes the grey checker (H 120, S 29) orange (H 14, S 99): **42 %** of the floor falls inside the box's HSV range. The box mask grows from 1,125 px (1,121 on the box) to **30,587 px, 29,864 of them on the floor**; in the debug image the light checker squares are drawn as box. Perception's shape guards reject such a blob.
2. **The box gets too dark.** Its brightness drops to V median **29**, below `hsv_box_v_min` = 40: only **11 %** of its pixels stay in range (99 % before).

The cube keeps V median 68 against its `hsv_cube_v_min` = 60 (81 % of its pixels in range): it survives, with little margin. The brain locks its target only with both the cube and the box, so it waits 10 s and stops with `ERROR:object_lost`, without moving: the failure is safe, but the task is not done.

Not fixed here (out of scope of #103: a weakness found by the demo becomes its own ticket). Brightness no longer separates the box from the floor (box V 25–50, floor V 26–43); saturation still does (box S 198–207, floor S 93–106). Leads for that ticket: a higher `hsv_box_s_min` with a lower `hsv_box_v_min`, a brightness-normalized threshold, or the camera's exposure and white balance on the real arm (`07` §4, step 6.4b).

**Fixed by #175** (MES-52): saturation floors between the lit floor and arm (S ≤ 113) and the box (S ≥ 164) and cube (S ≥ 205), V floors under their shaded sides (box S ≥ 140, V ≥ 20; cube S ≥ 150, V ≥ 30). Case 5 rerun on `fix/175-dim-light-box` (from `main` at `cdbd1d5`), the same machine, its own container: **5/5 cycles in `DONE` in each of 2 runs** under `dim`, cube and box on every image while the brain waits (9–13 images); the default-light bench 50/50.

## Limits and observations

- **3a, `VERIFY` with perception dead**: with no detection for 2 s, `VERIFY` ends in `DONE` with a warning, as `06` §3.1 specifies. A cube that left the table and a perception that stopped look the same; known and accepted.
- **3b stops perception** rather than hiding the cube from the camera: to the brain it is the same (nothing published), and moving the cube out of view would need a new sim service in the driver (out of scope).
- **Cases 1 and 2**: `/sim/randomize` moves the box too; the brain locks both again after `RETRY`.
- **A `/joint_states` stall on this machine**: twice in about 80 cycles, the brain saw no `/joint_states` for 0.5 s (two stacks sharing the CPU) or 1.36 s (the demo stack alone), and stopped with `ERROR:no_joint_states`, the arm frozen. The driver logged nothing. The nightly bench (Linux runner) has 50/50, so it may be specific to Windows + WSL2 + Docker Desktop (see the WSL2 clock steps of MES-37); not investigated here.
- **Perception does not always stop cleanly on SIGINT**: in one of the 15 runs that stop it (the last case-4 run), the node died with `RuntimeError: Unable to convert call argument '0'` in `take_message`, exit code 1, instead of a clean stop. The demo's outcome does not depend on it (perception is meant to stop there). It is the rclpy SIGINT race of AGENTS.md, which the driver handles since #158 and the perception node not yet.

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
