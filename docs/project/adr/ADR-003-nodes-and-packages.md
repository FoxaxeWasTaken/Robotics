# ADR-003 — Nodes and packages split

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (3 nodes) + DECIDED (shared packages) | 2026-10-08 | ARC → CTRL, KIN, VIS | C-03, C-05, C-08 |

## Context

Three nodes are imposed (driver, perception, brain). Constants (joint names, limits, gripper mapping) are needed by several nodes.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Constants duplicated in each node | No extra package | Silent divergences (limits, gripper mapping) |
| `so101_common` package (pure Python, no dependency) | Single source of truth, testable against the URDF | One more package |
| Read everything from the URDF at runtime | No duplication | Parsing everywhere; the gripper mapping is not in the URDF |

## Decision

The three imposed nodes + **`so101_common`** (constants, conversions) and **`so101_bringup`** (launch, YAML, RViz, tools) packages. Non-trivial logic lives in **pure modules** (`kinematics.py`, `fsm.py`, `detection.py`, `geometry.py`), ROS-free and testable.

## Consequences

- `so101_description` stays untouched (C-08).
- A test checks `so101_common` limits = URDF limits.

## Evidence / validation

Design review (`01` §2).

## Reconsider if

The number of nodes grows a lot (phase 2): consider composable nodes or a feature-based split.
