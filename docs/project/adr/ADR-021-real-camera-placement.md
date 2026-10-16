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
