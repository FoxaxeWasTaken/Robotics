# ADR-010 — Trajectory generation

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | KIN → CTRL, DEC | C-30, C-51 |

## Context

The current code sends the IK target in a single command; the driver's speed limiter smooths it (crude trapezoid, joints not coordinated).

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Jump + driver limiter (current) | Zero code | Uncoordinated joints; unpredictable TCP path |
| **Joint-space min-jerk streamed at 50 Hz by the brain** | Smooth, coordinated, controlled duration | A module to write |
| Cartesian (IK waypoints) | Guaranteed straight line | More IK; only useful for descent/lift |
| JointTrajectoryController (ros2_control) | Standard | `/joint_command` imposed; heavy integration |

## Decision

**Joint-space min-jerk** (quintic), duration computed from `v_max`, streamed on `/joint_command` at 50 Hz by the brain, starting from the **measured** position. Optional Cartesian descent/lift (5–10 waypoints) on the real arm. The driver limiter stays a **safety net** (ADR-023).

## Consequences

- End of motion: `max(abs(q − q_target)) < 0.01 rad` or a T + 1 s timeout.

## Evidence / validation

Pick & place spike (min-jerk profile): 39/40 [MES-09].

## Reconsider if

Obstacle-avoiding trajectories are needed: planning (OMPL / MoveIt) in phase 2.
