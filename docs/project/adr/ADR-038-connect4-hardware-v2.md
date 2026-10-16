# ADR-038 — Connect Four hardware: store-bought grid on a riser, depth guide, token handover, wrist camera

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (riser clearance measured) + RECOMMENDED (exact layout, from the purchased grid's measurements) | 2026-10-09 | KIN → CTRL, SAFE, VIS | C-41, C-51, C-54 |

## Context

ADR-035 chose a small custom grid with one funnel per column and a token magazine. Three inputs since then (2026-10-09): the team prefers a **store-bought grid** (nothing to build, robust, recognizable on video), whose slots are adjacent, which leaves no room for a per-column funnel; the teaching staff suggested **a camera on the gripper** for a close-up check of the alignment; and the first version will **receive its tokens from the human's hand**.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Small custom grid (ADR-035) | Clearance ≥ 10.8 cm with the arm on the table [MES-28] | To build; small tokens (Ø 20 mm) and cells |
| Store-bought classic grid, arm on the table | Nothing to build | 0.5 cm of clearance above the far column [MES-28] |
| **Store-bought classic grid, arm on an 8 cm riser**, columns at r = 0.14–0.36 m | All 7 columns reachable, clearance 6.3–10.5 cm [MES-32], jaw error 1–3° [MES-27]; big tokens and cells | A rigid riser clamped to the table; the table is 8 cm lower in `base_link` (phase-1 demos without the riser) |
| Travel-size grid | Maybe no riser | Smaller tokens: smaller self-centering capture and cells; dimensions differ between models |
| Per-column funnel | Captures the error in both directions | No room on a store-bought grid: the slots are adjacent |
| **Depth guide only** (a V across the grid, along the whole top edge) | Fits any grid; covers the tight direction | Along the grid, relies on the token rolling or sliding off the column wall (to measure) |
| Magazine or dispenser | Autonomous | To design and build; misfeeds |
| **Handover** (the human puts the token between the open jaws) | Nothing to build; the human is there anyway | A hand next to the gripper; the token's position in the jaws varies |
| **Wrist camera** (staff suggestion) | The token and the slot in the same image: a relative measurement, insensitive to calibration and model errors; also sees the token in the jaws | A second camera stream, a cable along the arm, a mount (optional part of the SO-ARM100 repository) |

## Decision

- **Store-bought classic grid**, still **edge-on** (ADR-035: placement, `drop_ik` and jig unchanged), **arm on a rigid riser of about 8 cm**, columns at about r = 0.14–0.36 m. The exact riser height and radius come from the purchased grid's measurements (`phase2/P2-04` step P2.1).
- **Depth guide**: two sloped strips along the top edge, a V across the grid only. No per-column funnel.
- **Token supply, first version: handover.** The gripper closes only on an explicit "token given" from the human, slowly, to a squeeze target just below the token thickness; the arm does not move until "done" (`phase2/P2-02` §5). A dispenser or a rack is a stretch feature.
- **Wrist camera**: planned step P2.8; **required** if the real drop bench stays below 90 %.
- Supersedes in ADR-035: grid size, funnel, magazine.

## Consequences

- Sim: the scene is generated from a copy of the provided one with `mujoco.MjSpec` (robot base raised, camera on the gripper body), ADR-036 update.
- Interfaces: `PickPlace` gets a pick mode for a token already held, plus `/brain/set_gripper`, `/connect4/token_given`, `/sim/connect4/give_token` and `/wrist_cam/*` (`phase2/P2-02` §6).
- Real setup: riser and arm clamped; the arm on the riser goes through the `07` §3 commissioning again.

## Evidence / validation

[MES-32] (`experiences/exp_connect4_kinematics.py`). `MjSpec` check (MuJoCo 3.15, 2026-10-09): on a copy of the provided scene, base raised by 8 cm and a camera added to the `gripper` body; the model compiles and the camera renders [CODE]. Still to measure: the purchased grid, the along-grid capture and handover offsets in the drop spike (P2.1), the real drop bench (P2.7).

## Reconsider if

The measured grid does not fit (far column out of reach, or clearance below 3 cm once link thickness is counted): travel-size grid or custom grid (ADR-035). The spike's along-grid capture is smaller than the arm error plus the handover offset: wrist camera before the real arm, or printed jaw pads with a recess that centers the token. The camera alone reaches ≥ 90 % in the real guide-vs-camera comparison (`phase2/P2-04` step P2.8): remove the depth guide. The camera is not expected to win: across the grid the slot leaves 1 to 2 mm per side, and one servo step moves the TCP by about 0.46 mm at 30 cm (4096 steps per turn), before gear backlash and sag. The guide also absorbs the kick the token gets when the jaws open.

## Update (2026-10-16, teaching staff)

Wrist camera: probably the WowRobo 2 MP USB module for SO-ARM100/101 (30 fps, 3 m cable), on a printable wrist mount (STL shared on Teams); some users fix a webcam directly. Availability per campus: the staff will confirm. The grid webcam can be any USB webcam (a phone only as a stopgap: latency).
