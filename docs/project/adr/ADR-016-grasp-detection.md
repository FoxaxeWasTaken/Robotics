# ADR-016 — Grasp detection and verification

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (revised after measurement) | 2026-10-08 | DEC → CTRL, VIS | C-30 |

## Context

We need to know whether the grasp succeeded to avoid carrying nothing. The initial proposal used the gripper position.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Gripper position | Available | **0 % with or without the cube**: the jaws go through the tunnels [MES-10] |
| **Gripper effort / load** | 0.12–0.19 normalized when holding vs 0 when empty | Different scale on the real arm: recalibrate |
| **Visual verification** (cube gone from the table / in the box) | Actuator-independent | Only with the arm out of view |

## Decision

The driver publishes the **normalized effort** in `JointState.effort` ([−1, 1]: sim `actuator_force / forcerange`, real `Present_Load / 1000`) — a documented deviation from REP-103. The brain requires effort above `grasp.effort_threshold` (0.06) after closing and during the lift, **then** a visual check in `VERIFY`.

## Consequences

- Threshold to recalibrate on the real arm (empty / holding measurements).
- `Present_Current` as a real-arm alternative if `Present_Load` is too noisy.

## Evidence / validation

[MES-10].

## Reconsider if

The real signal does not separate the cases: rely on visual verification only, or add a sensor.

## Update (2026-10-17, step 1.3)

Sign convention: the driver reports the gripper effort as its **closing force** (MuJoCo's raw value is negative when squeezing), so the brain keeps `effort > grasp.effort_threshold`. Measured in the team driver [MES-35]: 0.184 holding the cube, −0.001 empty; with the cube not turned to its tunnel, only 0.053, below the 0.06 threshold: the grasp planner must align the jaws with a tunnel.

## Update (2026-10-27, step 4.4a)

Threshold checked where the FSM judges it, on the nightly's 40 scenes [MES-48]: holding, the `GRASP` window never goes below 0.120 (aligned jaws or free roll), but the end of `LIFT` drops to 0.087, a ×1.45 margin instead of ×2; empty, 0.000. The 0.053 case of MES-35 never occurs with the planner. The threshold stays 0.06: it sorts all 120 grasps right, and lowering it would only shift the margin onto the empty side, exactly 0 in sim only. The real-arm recalibration (#69) checks both the `GRASP` window and the end of `LIFT`.
