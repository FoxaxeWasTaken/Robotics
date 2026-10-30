# ADR-005 — Frames, units and conventions

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (rad, robot frame) + DECIDED | 2026-10-08 | ARC → KIN, VIS | C-11, C-14, C-21 |

## Context

Positions must be expressed "in the robot frame"; angles in rad (REP-103) except the gripper in % on `/joint_command`. Measurements revealed convention pitfalls.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Publish in `world` | Frame of the provided marker | Less explicit than the robot frame |
| **Publish in `base_link`** (`world` = `base_link` through the provided identity TF) | Compliant with the brief; explicit | — |
| `use_sim_time` + `/clock` | Decouples clocks | Useless: the sim is paced in real time by the driver |

## Decision

- Working frame **`base_link`**; camera **`external_cam_optical_frame`** (optical convention z forward, y down).
- **`shoulder_pan` > 0 moves the TCP towards −y**: IK seeds `pan = −azimuth`, azimuth measured **from the pan axis (x = 3.88 cm)**.
- TCP orientation = URDF frame `gripper_frame_link` (the MuJoCo site is rotated by 90°).
- Wall clock, no `use_sim_time`.

## Consequences

- MuJoCo → OpenCV conversion: `R = R_mj @ diag(1, −1, −1)`.
- The current code publishes `frame_id = external_cam`: rename once the camera TF exists.

## Evidence / validation

[MES-02], [MES-04], [MES-06], [MES-13]; azimuth fix: pick & place 29/40 → 39/40 [MES-09].

## Reconsider if

A faster-than-real-time sim becomes useful (RL training in phase 2): switch to `use_sim_time`.

## Update (2026-10-30, #189)

Stamps stay on the ROS clock (they are compared across nodes), but the timers run on the **steady clock** and durations use `time.monotonic()`: on WSL2 the system clock steps back about 2 s every 30 s, which paused `/joint_states` and froze motions on the ROS clock [MES-37]. Still no `use_sim_time`.
