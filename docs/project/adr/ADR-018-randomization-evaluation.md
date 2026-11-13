# ADR-018 — Box randomization and evaluation tools

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (measures C-42) | 2026-10-08, updated 2026-10-09 | OPS → QA, CTRL | C-06, C-42 |

## Context

C-42 requires a **robust** pick & place in sim. Until its update of 2026-10-08 the brief named random object **and** box poses; the update dropped the wording, and we keep the protocol as our robustness measure (`00` I-11). `SO101Sim` only randomizes the cube and its code is installed non-editable.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Modify `sim/so101_sim` | Natural | Ignored without rebuild; conflicts with the staff's updates |
| **`/sim/randomize` service in the driver** (`model.body_pos` of `drop_box` + `randomize_object()`) | No change to `sim/`; usable by the bench | Internal sim access, confined to the driver |
| Script outside ROS | Simple | Does not test the real ROS stack |

## Decision

The driver exposes, **in sim only**, `/sim/randomize` and the `expose_ground_truth` parameter (**false by default**) publishing `/sim/ground_truth/*` for the **evaluation bench only**. The bench (`ros2 run so101_bringup evaluate`) runs N episodes and writes a CSV.

## Consequences

- CI guards: no ground-truth read in brain/perception (C-06).
- A ground-truth-fed perception stub is allowed **in development only**, never in a final launch file.

## Evidence / validation

Infrastructure review (`02` §2).

## Reconsider if

The teaching staff provides its own randomization: use it.

## Update (2026-10-16, teaching staff)

Confirmed: random cube **and** box poses are what is expected, and showing that it works on a few draws is enough. "Robust" refers to the five cases of [BOOT2 p12] (the object moves mid-action, the grasp fails, the object leaves the view, the target is out of reach, the lighting changes): show a quick example of how each one is handled, or not.

## Update (2026-10-23, step 4.3a)

The bench now runs on the real pipeline, so the switch reaches the launch, still off by default:

- `sim.launch.py` has an `expose_ground_truth` argument, default false. A `perception` argument, default false too, adds the real perception for a bench on its own `ROS_DOMAIN_ID`.
- The compose stack takes `EXPOSE_GROUND_TRUTH`, and only the E2E jobs set it.

The decision is unchanged: only the bench reads the ground truth, and the C-06 guards still check that no launch file outside `test/` enables it and that no node of the pipeline reads it.

## Update (2026-11-13, phase 2)

The bench also plays Connect Four games: `evaluate --games`, a scripted person (random, or the engine), the robot first in the even games, a seed per game, scored on the true board; a game fails on `FIX_BOARD`, `ERROR`, a lost token, a token in another column, an arm that moved during a person's turn, or the timeout (`08` §3.1, ADR-040). Target P2-06: 8 games of 10; measured 10 of 10 [MES-71].
