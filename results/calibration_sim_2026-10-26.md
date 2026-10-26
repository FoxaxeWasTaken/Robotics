# Extrinsic calibration in sim (step 3.4b, #45), 2026-10-26

Our calibration tool (`so101_perception.calibration`, 05 §5.1) on the calibration scene of #44, with the sim camera of `so101_bringup/config/sim_scene.yaml` (#47). The camera pose MuJoCo renders is the reference; perception never reads it (C-23).

## Protocol

- **Test** (`so101_bringup/test/test_sim_scene.py`): the scene rendered at 640×480 with the arm in PARK, `calibration.detect_markers` then `calibration.solve`, against the rendered camera.
- **Tool**: `ros2 launch so101_bringup sim.launch.py calibration:=true` on its own ROS domain, then `ros2 run so101_perception calibrate_extrinsics --markers src/so101_bringup/config/markers_sim.yaml --out <file>`: 10 frames, the arm at its start pose.

## Result

| Run | Markers | Mean reprojection | Position error | Rotation error |
| --- | --- | --- | --- | --- |
| Test, arm in PARK, corners without sub-pixel refinement | 8 | 1.17 px: **refused** (gate 1 px) | 1.24 mm | 0.062° |
| Test, arm in PARK, sub-pixel corners (kept) | 8 | **0.87 px** | **0.53 mm** | **0.024°** |
| Tool, arm at its start pose | 6 (2 hidden by the arm) | 0.887 px | 0.75 mm | 0.056° |
| Tool, a 0.1 px gate | 6 | refused, exit 1, no file written | – | – |

Step 3.4b's criterion (< 5 mm / 0.3°) is met, as MES-16 expected (0.9 mm / 0.06°).

The margin under the 1 px gate is small (0.87 px): the rendered markers' corners stay up to ~3 px off at grazing angles (`test_sim_scene.py`). The pose stays sub-millimeter: a larger error comes from a wrong layout, which the gate is there to catch.
