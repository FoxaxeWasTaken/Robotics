# Perception robustness in sim (step 3.5, #47), 2026-10-26

`docs/project/experiences/exp_perception_robustness.py` (MES-46), next to `ros2 launch so101_bringup sim.launch.py expose_ground_truth:=true perception:=true` on its own ROS domain, with the sim camera moved by #47 (`so101_bringup/config/sim_scene.yaml`, MES-45). The ground truth only scores perception (C-06).

## Protocol

- 200 scenes drawn by `/sim/randomize` (driver seed 0), the arm in PARK, as when the brain locks its target.
- Once the cube has settled, 2 s of `/object_position`, each pose against the ground truth.
- Every 10th scene, `/go_to_target` 10 cm from the cube towards the camera, then back to PARK: the arm passes between them. A scene where the arm pushed the cube is dropped.
- An outlier: a published cube center more than 2 cm from the ground truth (step 3.5's criterion).

## Result

| Case | Scenes | Cube published | Poses | Mean | p95 | Max | Above 2 cm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Arm in PARK | 200 | **200 (100 %)** | 3,838 | **0.7 mm** | 1.3 mm | 2.8 mm | **0** |
| Arm between the camera and the cube | 14 (6 dropped, cube pushed) | 921 poses in about 1,148 images | 921 | | | **17.4 mm** | **0** |

Step 3.5's criterion is met: no published position off by more than 2 cm. Before the camera move, 21 % of the cubes were cut by the image edge and never published (MES-12): the publication rate goes from about 79 % to 100 %.

## Pick & place

`ros2 run so101_bringup evaluate --episodes 50` next to the same stack, driver seed 0, so the scenes of the nightly bench: **50/50**, duration p50 14.5 s, p95 17.5 s. The nightly of 2026-10-24 on `main`, with the provided camera: 40/50, the 10 failures `object_lost` (cubes cut by the image edge).

## Limit

A cube the arm partly hides is still published, its center pulled towards its visible part: up to 17.4 mm here, and 20.5 mm when the arm stayed in front of the cube for a whole scene (an early run of the script, before it checked PARK). Perception only rejects a cube cut by the image edge. The brain locks its target in PARK, where nothing hides the cube (MES-45), and does not read perception while it moves, so the pick & place is not affected; a check of the blob area against the cube's expected area at that distance would close the gap.
