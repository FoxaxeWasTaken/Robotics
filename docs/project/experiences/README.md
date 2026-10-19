# Experiments (spikes) — reproducible evidence

These scripts are **throwaway**: they answer technical questions by measurement and feed the decisions (ADRs) and the register in `../11_cross_review_and_measurements.md`. **This is not production code**: project code is rewritten cleanly, with tests, in `ros2_ws/src`.

⚠ `exp_pick_place_gt.py`, `exp_perception_calib.py` and `exp_calibration_methods.py` use the sim **ground truth** (`get_object_position`, `get_camera_extrinsics`) to isolate a subsystem: allowed in tests, **forbidden in the pipeline** (C-06, C-23).

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
| `exp_park_pose.py` | Is PARK reached within 0.01 rad through the actuators? Does the parked arm hide the cube or the box? | MES-39 | < 1 min |
| `exp_webcam_formats.py` | Which webcam formats go through usbipd? (run with `--device /dev/video0`) | MES-17 | < 1 min |
