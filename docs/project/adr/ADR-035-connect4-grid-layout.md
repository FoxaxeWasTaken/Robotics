# ADR-035 — Connect Four: small edge-on grid, funnel, magazine, drop poses with locked wrist roll

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (kinematics measured); grid, funnel and magazine **superseded by ADR-038** | 2026-10-09 | KIN → CTRL, SAFE, VIS | C-41, C-54 |

## Context

A token can only enter a slot if it stays in the grid plane: the jaw closing axis (x of `gripper_frame_link`) must be normal to the grid. The arm has 5 DoF, its approach axis always lies in the vertical plane through the pan axis (`04` §1), and the real accuracy target is 1 cm (C-41) while a slot leaves 1 mm per side.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Grid **facing** the robot | Natural for a human player | Needs a vertical approach: best jaw error 40–62° at 22–30 cm high, 17–23° at 14 cm [MES-27] |
| **Grid edge-on** (its plane contains the pan axis) | Jaws normal to the arm plane = wrist roll ±90°: 1–3° over r = 14–36 cm, z = 6–34 cm [MES-27] | The camera and the human see the grid from the side; the arm works above the grid |
| Standard store-bought grid | Nothing to build | Slots at about 25 cm: 0.5 cm of clearance above the far column, link thickness not counted [MES-28] |
| **Small custom grid** (slots at 15 cm, columns at r = 0.16–0.30 m) | Clearance ≥ 10.8 cm [MES-28]; modeled exactly in sim | To build |
| Drop poses with ikpy `orientation_mode="X"` | One call | 11 % of the targets solved, even with 15 seeds [MES-29] |
| **Wrist roll locked at ±90°, position IK on 4 joints** (`drop_ik`) | Every reachable target solved [MES-29]; plain phase-1 position IK | 2.8° constant residual from the URDF wrist [MES-29]; 0.26 s median per pose |

## Decision

- **Small custom grid, edge-on**: 7 × 6 cells, pitch 24 mm, columns at r = 0.160–0.304 m from the pan axis, slot entries at about 15 cm; 6 mm slots for 20 × 4 mm tokens.
- **A funnel** on top of the grid (entry about 22 × 20 mm per column) to absorb the real-arm error.
- **A magazine** whose tokens stand in their own edge-on plane: the pick uses the same jaw orientation as the drop.
- **Drop and pick poses with `drop_ik`** (`phase2/P2-02` §2.2), checked by FK; ikpy stays the solver (ADR-008).

## Consequences

- The robot works along the edge of the grid; the camera faces one face of the grid and the human plays from the other face.
- The arm moves above the grid: drop poses are approached from above, through a pre-drop pose 3 cm higher.
- Placement tolerance of the grid: a yaw error adds directly to the jaw error; keep it within ±3° (printed jig).

## Evidence / validation

[MES-27], [MES-28], [MES-29] (`experiences/exp_connect4_kinematics.py`). Still to measure: drop success in sim (spike P2.1) and on the real arm (P2.7).

## Reconsider if

The drop spike fails with the funnel (→ level 0, ADR-034), or the real jaw error exceeds the slot clearance (wider slots, deeper funnel).

## Update (2026-10-09, ADR-038)

Kept: edge-on placement, `drop_ik`, placement jig. Superseded: the small custom grid becomes a store-bought grid with the arm on an 8 cm riser [MES-32]; the per-column funnel becomes a depth guide only (store-bought slots are adjacent); the magazine becomes a handover from the human, with a dispenser as a stretch feature; a wrist camera is added (staff suggestion).
