# Experiments (spikes) — reproducible evidence

These scripts are **throwaway**: they answer technical questions by measurement and feed the decisions (ADRs) and the register in `../11_cross_review_and_measurements.md`. **This is not production code**: project code is rewritten cleanly, with tests, in `ros2_ws/src`.

⚠ `exp_pick_place_gt.py`, `exp_perception_calib.py` and `exp_calibration_methods.py` use the sim **ground truth** (`get_object_position`, `get_camera_extrinsics`) to isolate a subsystem: allowed in tests, **forbidden in the pipeline** (C-06, C-23).

⚠ `connect4/` holds the prototypes of the phase-2 review of 2026-11-07, as they ran: its [README](connect4/README.md) says what each folder measures and how to run it. They use the sim ground truth to score, and `reading/` reads the true grid pose: tests only (C-06).

## Running

From the repository root (with the `tek5/control` image built):

```bash
docker run --rm -e MUJOCO_GL=osmesa \
  -v $PWD/ros2_ws:/ros2_ws:ro -v $PWD/sim:/opt/so101/sim:ro \
  -v $PWD/docs/project/experiences:/exp -v /tmp/exp_out:/out \
  tek5/control bash -lc "source /opt/ros/jazzy/setup.bash && python3 /exp/<script>.py"
```

Produced images (camera views, masks, ArUco) are written to `/tmp/exp_out`.

## Scripts

| Script | Question | Measurements | Indicative duration |
| --- | --- | --- | --- |
| `exp_kinematics.py` | Sim step; do URDF and MuJoCo agree? TCP axes? Pan sign? | MES-02, 04, 05, 06 | < 1 min |
| `exp_reachability.py` | Where can the gripper point straight down? Minimal tilt as a function of (z, r) | MES-08 | ~5 min |
| `exp_ikpy_vs_pinocchio.py` | Is ikpy enough? Comparison with Pinocchio DLS (success, time) | MES-07, 11, 21 | ~2 min |
| `exp_pick_place_gt.py` | Does the grasp strategy work (40 episodes, ground truth)? Which grasp signal? | MES-05, 09, 10 | ~10 min |
| `exp_perception_calib.py` | Field of view, HSV detection, pixel → 3D error, yaw, box, ArUco calibration | MES-12 to 16 | ~2 min |
| `exp_calibration_methods.py` | 1 ArUco (demo) vs several, with detection noise | MES-25 | ~2 min |
| `exp_connect4_kinematics.py` | Phase 2: can the arm drop a token into a vertical grid? Edge-on vs facing, clearance (with and without a riser), drop-pose method, cell size in the image | MES-27 to 29, 31, 32 | ~3 min |
| `exp_driver_speed.py` | How fast does a joint move behind the driver's speed limiter (sim time, with and without it)? | MES-33 | < 1 min |
| `exp_calibration_zero.py` | Real arm: is LeRobot's zero the URDF zero? FK of the rest pose under both hypotheses | MES-30 | < 1 min |
| `exp_zero_offsets.py` | Real arm: each joint's reading at the URDF zero, read-only: the `joint_zero_offsets` line | MES-55 | < 1 min |
| `exp_gripper_load.py` | Real arm: the gripper's load open, closed empty and on the cube (only the gripper moves) | MES-56 | 1 min |
| `exp_park_pose.py` | Is PARK reached within 0.01 rad through the actuators? Does the parked arm hide the cube or the box? | MES-39 | < 1 min |
| `exp_gt_cycles.py` | Does a full pick & place cycle run in sim with the ground-truth stub, and how long does it take? What does planning cost the brain's tick? | MES-40 | 6 min (20 cycles) |
| `exp_ik_self_collision.py` | Which position-IK solutions collide with the arm itself? From which seed does `test_ik_accuracy` get none? | MES-43 | < 1 min |
| `exp_tick_by_state.py` | The brain's tick period per FSM state, the planner in a worker process or thread (patches a copy of the node) | MES-40 | with `exp_gt_cycles.py` |
| `exp_principal_point.py` | Is the sim's principal point in OpenCV pixel coordinates? Blob against the exact silhouette, both conventions | MES-41 | < 1 min |
| `exp_webcam_formats.py` | Which webcam formats go through usbipd? (run with `--device /dev/video0`) | MES-17 | < 1 min |
| `exp_sigint_stop.py` | Does a SIGINT at a random moment make a node print a traceback at exit, with `rclpy.spin()` and with an event-driven stop? | MES-44, MES-50 (`--perception`) | ~12 min (2 × 150 stops) |
| `exp_camera_pose.py` | Where to put the sim camera so that every cube and box of `/sim/randomize` is in full view? Best pose per sector by projection, then rendering: cubes cut by the image edge, what the arm in PARK hides | MES-45, MES-53 (the hidden cubes) | ~30 s |
| `exp_perception_robustness.py` | With the moved camera, how far off and how often does perception publish, arm in PARK and arm between the camera and the cube? (run next to a stack with the ground truth and perception) | MES-46 | ~15 min (200 scenes) |
| `exp_driver_jitter.py` | The driver's control period with the sim camera thread on and off, and with llvmpipe's render threads at half the cores (`LP_NUM_THREADS`)? | MES-34 | 30 s per variant |
| `exp_ik_seeds.py` | Does `ik_pos` find every reachable configuration of the joint space with the seeds of `04` §3.1? (2000 random configurations, target = FK) | MES-36 | ~5 min (2 × 2000 solves; a miss tries every seed) |
| `exp_clock_steps.py` | How often does the WSL2 wall clock step, and what does a step do to `/joint_states` and to a `/go_to_target` motion? (run next to a sim stack) | MES-37 | 95 s by default (argument) |
| `exp_grasp_effort.py` | Where does the gripper effort sit when the FSM judges it (`GRASP` window, end of `LIFT`), holding the cube with aligned jaws or free roll, and empty? Is `grasp.effort_threshold` well placed? | MES-48 | < 1 min (3 × 40 grasps) |
| `exp_grasp_position.py` | Can the measured gripper opening tell a held cube from an empty close (the real arm's signal, #69)? Same scenes and mechanics as `exp_grasp_effort.py` | MES-57 | ~5 min |
| `exp_fault_scenarios.py` | Fault scenarios 1 and 2 of step 4.4a: the cube moved during `APPROACH` (empty grasp, `RETRY`, a new lock), the perception container stopped during `TRANSPORT` (the cycle completes)? (run against the stack, commands in its docstring) | — (#54) | ~30 s per scenario |
| `exp_lighting_hsv.py` | Why does the box vanish under the robustness demo's dim light? The box's and cube's HSV under the default and dim lighting, against perception's thresholds | MES-49, MES-52 (`[N]` placements) | < 1 min |
| `exp_joint_states_gaps.py` | Who stalls `/joint_states` on a running stack, the driver or the transport? Every gap with its arrival and stamp gaps (run next to a sim stack) | MES-51 | `[minutes]` |
| `exp_recorder_joins.sh` | Does a DDS participant that joins hold the driver's executor? Rosbag recorders started on an idle sim stack, the driver's late ticks counted (a container on 4 cores) | MES-80 | `[minutes]` |
| `exp_image_loss_under_load.sh` | Does a machine short of CPU lose perception's best-effort images? Perception's launch test in a loop beside six busy loops, the kernel's dropped UDP datagrams counted (a container on one core) | MES-82 | `[minutes]` |
| `exp_cube_occlusion.py` | How much of a cube must perception see to publish it? The blob against a whole cube's silhouette for whole cubes and cubes hidden by the arm or the box's walls, with the center error | MES-53 | not timed |
| `exp_sim_calibration_render.py` | Does the sim calibration view render the same on every machine? Image hash, reprojection and worst corner per llvmpipe setting, then `[N]` sim seeds | MES-54 | not timed; one process per llvmpipe setting |
