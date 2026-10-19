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

## Update (2026-10-19, steps 2.1b, 2.4a, 2.4b)

Implemented in `so101_brain/grasp_planner.py` and validated in closed loop by the repository code (`so101_bringup/test/test_grasp_closed_loop.py`): 40/40 on 3 seeds, cube **and** box randomized [MES-38]. Two points of the decision change, and one is added:

- **Drop**: the gripper as vertical as reachable (the grasp tilts, 0 to 30°), instead of a free orientation. With a free orientation, 2/40 cubes stayed hooked on a jaw inside one of their tunnels, the gripper tilted 62–68°. Position only stays the last resort.
- **Lift and drop keep the wrist roll** of the previous pose: left free, the position-only IK spun it 2.3–4.1 rad for a 6 cm lift, seconds of motion. `/go_to_target` keeps the free roll (`04` §7).
- **Of the 4 face candidates**, the one with the shortest first motion, not the first found.
