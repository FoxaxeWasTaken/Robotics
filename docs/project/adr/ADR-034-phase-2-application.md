# ADR-034 — Phase-2 application: Connect Four against the robot

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED, **validated by the teaching staff** on 2026-10-09 (step 7.1) | 2026-10-09 | CP → all | C-53, C-54 |

## Context

Phase 2 is mandatory at the final defense (C-54): an application of the team's choice that reuses the phase-1 work, feasible in a few weeks, demonstrable on video, a real robotics skill, within the workshop constraints ([BOOT2 p15]: 1 arm with 5 DoF, low payload, tabletop, 1 camera + additions, ROS 2 and sim first, slow near humans). The team also wants a project that reads well in a public repository.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| **Connect Four against the robot** | Understood in 3 s on video; reuses every phase-1 block; 3 natural lanes (perception, brain, hardware); playable in sim by anyone | Drop precision into a vertical grid; a 5-DoF arm cannot face the grid [MES-27] |
| Tic-tac-toe with the cubes | Lowest risk: phase-1 pick & place on a 3×3 grid | Little new robotics; listed as an example in [BOOT2 p16] |
| Voice or natural-language commands (LLM) | Trendy; reuses brain and perception | Open-vocabulary grasping of arbitrary objects; latency; the robotics part is mostly phase 1 |
| Portrait drawing with a pen | Very shareable | Contact control and backlash; reuses less of the perception pipeline |
| Shell game (cups) | Fun | Tracking under hand occlusion is the whole risk |
| Imitation learning (ACT / SmolVLA) trained on phase-1 demos | Strong ML angle | GPU, sim-to-real of a visual policy: highest risk |

## Decision

**Connect Four against the robot**, with tic-tac-toe as the **level-0 fallback** if the drop spike fails (`phase2/P2-01` §2): same software, no vertical grid. Design: `phase2/`.

## Consequences

- Hardware to build: small custom grid, funnel, token magazine, placement jig (ADR-035).
- New package `so101_connect4`, a `PickPlace` action in the brain (ADR-037), a phase-2 sim scene (ADR-036).
- The first phase-2 work is the drop spike (`phase2/P2-04` step P2.1).

## Evidence / validation

[MES-27], [MES-28], [MES-29], [MES-31] (`experiences/exp_connect4_kinematics.py`); ikpy is enough [MES-21], reproduced on 2026-10-09. **Staff validation: 2026-10-09**, project accepted. The questions of `phase2/P2-03` §4 are still to ask.

## Reconsider if

The drop spike is a no-go (→ level 0, tic-tac-toe), or the staff changes the phase-2 requirements.

## Update (2026-10-09, ADR-038)

Hardware changed: store-bought grid on a riser, depth guide, tokens handed over by the human, wrist camera (ADR-038). The plan moved to its own roadmap, `phase2/P2-04` (the drop spike is still step P2.1).

## Update (2026-10-16, staff answers)

The questions of `phase2/P2-03` §4 were asked and answered on 2026-10-16: the answers are there.

## Update (2026-11-07, scope of the hand-in)

The hand-in is 2026-11-20 23:42. **`v2.0.0` is level 1, the game in simulation; level 2, the real arm, is `v2.1.0`** (`phase2/P2-01` §2). Phase 1 already runs on the real arm (`v1.3.0`), and a real game adds a riser, a handover next to a person and a new camera setup: an expert review found five blocking safety points and a drop precision still unproven (`phase2/P2-02` §8), too much for two weeks. The teaching staff validated a pitch aiming at the real arm: the defense says so plainly.

A drop-spike no-go first changes the sim geometry, a parameter; tic-tac-toe stays the last fallback. The spike's prototypes already pass [MES-63], [MES-64]; #76 decides on one layout. `PickPlace` became a `/brain/drop` service (ADR-037 Update).
