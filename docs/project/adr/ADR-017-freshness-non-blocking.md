# ADR-017 — Data freshness and non-blocking control

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED + DECIDED | 2026-10-08 | DEC → ARC, VIS | C-33 |

## Context

"Control never blocks waiting for perception (last known value)." Perception loses the object when the arm occludes it.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Wait for a fresh detection at every step | Up-to-date data | **Violates C-33**; blocks during occlusions |
| **Lock the target with the arm out of view, then use it without waiting** | Robust to occlusions | If the object moves, only detected in `VERIFY`/`RETRY` |
| Continuous visual tracking | Reactive | Occlusions; complexity |

## Decision

Callbacks store `(msg, stamp)`; the target is **locked** in `WAIT_TARGET` (5 consistent detections, arm in PARK), then the **last known value** is used; long computations use futures; `/joint_states` missing for > 0.5 s → `ERROR`; no `sleep` in callbacks.

## Consequences

- Test: perception killed during `TRANSPORT`, the cycle still completes.

## Evidence / validation

Design review (`06` §4).

## Reconsider if

Moving objects (phase 2): tracking and replanning.
