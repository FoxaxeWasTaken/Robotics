# ADR-022 — Observability: RViz, debug, recording

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (RViz, state) + DECIDED | 2026-10-08 | ARC → QA, DEC, VIS | C-24, C-34, C-52 |

## Context

We must be able to show and diagnose: robot, TFs, image and mask, brain state.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Logs only | Nothing to do | Slow diagnosis |
| **Versioned RViz + `/perception/debug_image` + `/brain/markers` + `/brain_state` + rosbag2** | Everything visible and replayable | Some visualization code |

## Decision

Versioned RViz config (RobotModel, TF, raw and debug Image, object/box Pose, box and target Markers), `/brain_state` transient_local, one rosbag2 per failed evaluation episode.

## Consequences

- Optional Camera display to check the calibration visually.

## Evidence / validation

Slides [BOOT p9], [KO2 p19, p26].

## Reconsider if

—
