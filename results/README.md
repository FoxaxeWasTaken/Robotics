# Results

Evaluation reports versioned at each milestone (lightweight CSV/MD). Rosbags stay out of Git (`docs/project/15` §7).

| Report | What it measures |
| --- | --- |
| [`ik_accuracy_sim_2026-10-18.md`](ik_accuracy_sim_2026-10-18.md) | The arm reaches a point in space, in simulation (C-41) |
| [`calibration_sim_2026-10-26.md`](calibration_sim_2026-10-26.md) | The camera's pose from our own calibration, in simulation |
| [`perception_robustness_sim_2026-10-26.md`](perception_robustness_sim_2026-10-26.md) | The cube and the box found under changed lights and views |
| [`bench_sim_2026-10-27.md`](bench_sim_2026-10-27.md), its CSV | 50 pick & place episodes in simulation |
| [`bench_real_2026-11-06.md`](bench_real_2026-11-06.md) | The pick & place on the real arm |
| [`robustness_demo.md`](robustness_demo.md) | The fault cases, one command each: five of the pick & place, eight of a Connect Four game |
| [`phase2_2026-11-13.md`](phase2_2026-11-13.md), its CSV | Connect Four, the first nightly on the game's code: the games, the coverage, the engine's strength |
| [`phase2_2026-11-14.md`](phase2_2026-11-14.md), its CSV | Connect Four as it ships: the tokens taken at the dispenser, the positions from perception |
