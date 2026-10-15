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
