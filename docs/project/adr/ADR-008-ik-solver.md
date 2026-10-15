# ADR-008 — IK solver: ikpy (Pinocchio as an option)

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (revised after measurement) | 2026-10-08 | KIN → CP, QA | C-31, C-41 |

## Context

The brief suggests ikpy or Pinocchio. The team already implemented FK/IK with **ikpy**. The first version of this folder recommended Pinocchio; the review (RC-19) demanded a measured comparison.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| **ikpy** | Already in place; 100 % position, 97 % vertical pointing; X/Y/Z/all orientation modes | 6–25 ms per solve |
| Pinocchio (hand-written DLS) | 20–50× faster; foundation for Pink (QP) | IK code to write and maintain; 91 % pointing with a single seed |
| Analytical IK | Deterministic, all solutions | Geometric offsets derived by hand; fragile if the URDF changes |
| MoveIt 2 / TRAC-IK | Industry standard | Heavy integration; out of scope |

## Decision

**ikpy**, behind a `Kinematics` façade (`04` §9) allowing a switch to Pinocchio without touching the rest. Whatever the solver: **multi-start** with `pan = −azimuth`, **FK verification** (reject above 1 mm, 1 cm for `/go_to_target`), never execute an unconverged solution.

## Consequences

- Planning in a worker thread (a few hundred ms worst case).
- Pinocchio stays available in the image if speed becomes critical.

## Evidence / validation

[MES-07], [MES-21] (`exp_ikpy_vs_pinocchio.py`); `/go_to_target` through the merged driver: 0.13 mm [MES-24].

## Reconsider if

Planning exceeds 1 s, or extra constraints appear (avoidance, task priorities): switch to Pinocchio + Pink.

## Update (2026-10-09)

- [MES-21] reproduced identically.
- Pinocchio 4.1.0 comes from `ros-jazzy-pinocchio`: import it after `source /opt/ros/jazzy/setup.bash`.
- Phase 2 needs a jaw-axis constraint: wrist roll locked at ±90° + position IK solves every reachable target, ikpy `orientation_mode="X"` only 11 % [MES-29] (ADR-035). ikpy stays the solver.
