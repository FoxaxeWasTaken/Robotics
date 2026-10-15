# ADR-009 — Grasp strategy

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | KIN → DEC, VIS | C-30 |

## Context

A 5-DoF arm cannot point straight down everywhere. The 5 cm cube is pierced by diamond tunnels that the scene's collisions leave open to the gripper.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Strictly vertical everywhere | Simple | 58 % reachability [MES-08] |
| **Minimal tilt (0 → 30°) for grasping, position-only for transport** | 39/40 in sim [MES-09] | Richer planning |
| Mandatory jaw alignment | "Clean" grasp | Infeasible with 10° tilt for 14/40 cubes; unnecessary in sim (grasp tolerant to 44°) |
| **Best-effort alignment, free fallback** | Robust | Less centered grasp in fallback |

## Decision

Grasp: TCP at the cube center, approach **as vertical as possible** (0, 10, 20, 30°) in the arm plane, jaws aligned to a face when possible (4 candidates, 90° symmetry), otherwise free roll. Pre-grasp **4 cm back along the approach axis**. Lift +10 cm, transport and drop (TCP ≈ 9 cm above the floor) with **position-only IK**.

## Consequences

- The cube yaw estimated by perception helps but is not critical in sim.
- Drop verification is mandatory (ADR-016).

## Evidence / validation

[MES-08], [MES-09] (`exp_pick_place_gt.py`).

## Reconsider if

On the real arm misaligned grasps often fail: make alignment mandatory and reduce the reachable area, or add a pre-positioning step.
