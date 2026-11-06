# ADR-021 — Real camera: placement and settings

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| RECOMMENDED | 2026-10-08 | VIS → CTRL, CP | C-43 |

## Context

In sim, 21 % of the cube positions are partly out of view; on the real arm auto-exposure changes perceived colors.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Placement by eye | Fast | Truncation and occlusion risk |
| **Placement checked by projection** (area + box + markers visible with margin) + locked settings | No truncation; stable HSV | Some preparation |
| Overhead vertical camera | Fewer occlusions, simple geometry | Bulkier mount; different from the sim |

## Decision

Rigid mount, viewpoint close to the sim (high, on the side, 30–40° down), **whole area visible with margin**, **manual exposure, white balance and focus** (`v4l2-ctl`), MJPG 640×480.

## Consequences

- Recalibrate the extrinsics if the camera moves; startup self-check.

## Evidence / validation

[MES-12], [MES-17].

## Reconsider if

The teaching staff imposes a standard camera mount.

## Update (2026-10-26, #47)

The sim camera moved, so "a viewpoint close to the sim" now means its new pose (`so101_bringup/config/sim_scene.yaml`): (0.458, 0.339, 0.363) in `base_link`, on the provided camera's side (azimuth 50° around the work area), 35° down, 0.59 m from the center of the work area, so lower and closer than the provided (0.45, 0.45, 0.55). It keeps every cube and box of `/sim/randomize` ≥ 10 px inside the image, where the provided pose cut 19 % of the cubes; the arm in PARK hides neither; and a cube lying in the box stays readable over its walls, which 30° down did not ensure [MES-45]. The decision itself is unchanged: rigid mount, the whole area in view with margin, manual exposure, white balance and focus, MJPG 640×480. The real camera gets placed alike and calibrated (6.3a).

## Update (2026-11-06)

The real bench does not copy the sim's viewpoint: the webcam stands 62 cm above the table, 52° down, on the arm's side, because that frames the working area at 640×480 (`07` §9 step 2). The rest holds: a rigid mount and locked settings.
