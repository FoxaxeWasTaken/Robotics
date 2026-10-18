# IK accuracy in sim (C-41), 2026-10-18

`colcon test --packages-select so101_bringup` → `test/test_ik_accuracy.py` (protocol: `docs/project/04` §7). Driver and brain from `so101_bringup/config` (`motion.v_max_rad_s` 1.0), sim scene of the course kit. True TCP: MuJoCo FK (`gripperframe` site) of the measured `/joint_states`, independent of the ikpy/URDF model.

## Result

| Targets | Mean | p95 | Max | C-41 |
| --- | --- | --- | --- | --- |
| 20 | **0.165 mm** | **0.228 mm** | **0.252 mm** | < 10 mm: met |

Gripper commanded 0, 50, 100 %: measured within 0.02 %, 1.80 %, 0.39 % over 15 `/joint_states` (bound: 3 %).

Four runs on 2026-10-18 gave a max of 0.221 to 0.252 mm. The TCP stays about 0.15 mm low: the position servos sag under gravity.

## Targets

Seeded (41): r 0.16–0.30 m from `base_link`, azimuth −90° to 60°, z 3–12 cm. The cube and the box are obstacles at these heights: `/sim/randomize` (driver `seed` 0) put the cube at (0.283, −0.077) and the box at (0.087, −0.246), and targets closer than 8 cm (cube) or 12 cm (box) were drawn again.

| # | Target x, y, z (m) | Measured x, y, z (m) | Error (mm) |
| --- | --- | --- | --- |
| 1 | 0.2657, 0.1250, 0.0413 | 0.2656, 0.1249, 0.0412 | 0.217 |
| 2 | 0.2192, 0.1674, 0.0615 | 0.2191, 0.1673, 0.0613 | 0.198 |
| 3 | 0.2316, 0.0015, 0.0605 | 0.2315, 0.0015, 0.0604 | 0.146 |
| 4 | 0.2323, 0.0840, 0.1075 | 0.2323, 0.0840, 0.1074 | 0.151 |
| 5 | 0.1869, -0.0896, 0.0835 | 0.1869, -0.0896, 0.0834 | 0.121 |
| 6 | 0.1939, 0.2245, 0.0353 | 0.1938, 0.2244, 0.0351 | 0.227 |
| 7 | 0.2155, 0.1094, 0.0456 | 0.2155, 0.1093, 0.0455 | 0.163 |
| 8 | 0.2291, -0.1773, 0.1054 | 0.2291, -0.1773, 0.1052 | 0.202 |
| 9 | 0.2008, 0.0590, 0.1092 | 0.2007, 0.0590, 0.1091 | 0.122 |
| 10 | 0.1552, -0.1341, 0.0525 | 0.1552, -0.1340, 0.0524 | 0.132 |
| 11 | 0.2535, 0.1168, 0.1076 | 0.2535, 0.1168, 0.1074 | 0.184 |
| 12 | 0.1733, 0.1631, 0.0417 | 0.1733, 0.1630, 0.0416 | 0.164 |
| 13 | 0.2340, -0.1600, 0.0673 | 0.2340, -0.1600, 0.0671 | 0.200 |
| 14 | 0.2613, 0.0904, 0.0347 | 0.2612, 0.0903, 0.0346 | 0.195 |
| 15 | 0.1299, 0.1459, 0.0395 | 0.1299, 0.1458, 0.0393 | 0.132 |
| 16 | 0.2364, 0.1434, 0.0341 | 0.2363, 0.1434, 0.0340 | 0.182 |
| 17 | 0.1512, 0.1996, 0.0927 | 0.1513, 0.1997, 0.0929 | 0.252 |
| 18 | 0.1690, 0.0722, 0.1170 | 0.1689, 0.0722, 0.1169 | 0.092 |
| 19 | 0.1407, 0.0948, 0.0748 | 0.1406, 0.0947, 0.0747 | 0.098 |
| 20 | 0.2058, -0.0452, 0.0496 | 0.2057, -0.0452, 0.0495 | 0.130 |
