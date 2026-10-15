# ADR-001 — ROS 2 Jazzy and Python (rclpy)

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (ROS 2) + DECIDED (Python) | 2026-10-08 | ARC → OPS, KIN | C-01 |

## Context

The brief imposes ROS 2; the provided images ship Jazzy (Ubuntu 24.04). The node language remains to be chosen.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Python (rclpy) | All key libraries are Python (SO101Sim, LeRobot, ikpy, OpenCV); fast iteration | Lower raw performance (no impact here) |
| C++ (rclcpp) | Performance, real time | Rewrite the Python drivers, compilation, learning curve |
| Mixed | Best of both | Two toolchains for a team of 3 |

## Decision

**Python everywhere** (`ament_python` packages), except `so101_interfaces` (`ament_cmake`, required by rosidl generation).

## Consequences

- Compute budget is ample: detection 0.5 ms/frame, IK 6–25 ms, 0.1 ms per backend call.
- Mind the GIL: heavy work (rendering, planning) goes to threads that release the GIL (MuJoCo) or outside the tick.

## Evidence / validation

[MES-11], [MES-13], [MES-20].

## Reconsider if

High-rate control (> 200 Hz) or heavy real-time image processing appears (phase 2).
