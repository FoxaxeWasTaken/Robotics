# ADR-011 — Detection: HSV thresholding

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (justified method) + DECIDED | 2026-10-08 | VIS → QA, ARC | C-20, C-24 |

## Context

The object is a saturated red cube of known shape; the box is brown on a gray floor. The method is free but must be justified.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| **HSV + morphology + contours** | 0.5 ms/frame, no data, explainable; 0 false positive in sim | Sensitive to real lighting |
| YOLO / learned segmentation | Robust with a varied dataset | Sim + real dataset to annotate; 10–50 ms CPU |
| ArUco on the object | Direct 6D pose | Changes the imposed object |
| Open-vocabulary detectors | No training | Heavy, latency, overkill |

## Decision

HSV: cube **H ∈ [160, 180] ∪ [0, 8]** (red wraps around 0/180), S ≥ 100, V ≥ 60; box H ∈ [5, 25], S ≥ 90. Guards: area, solidity, **truncation detection**. Thresholds as ROS parameters; debug image published.

## Consequences

- The provided stub (yellow golden-ball thresholds) is rewritten.
- Real arm: locked camera settings, thresholds tuned on recorded real images (ADR-021).

## Evidence / validation

[MES-13], [MES-15]; real PLA color noted in the scene (HSV ~168, 185).

## Reconsider if

Real thresholds fail despite locked settings, or phase 2 brings multiple/non-colored objects: YOLO-seg.
