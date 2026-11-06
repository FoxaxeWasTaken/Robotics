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

## Update (2026-10-30, #189)

The ranges of the decision held under the default light only. Under the robustness demo's dim, warm light the floor fell in the box's range and the box under its V floor [MES-49]. The ranges are now saturation floors well above the lit floor and arm (S ≤ 113) and V floors under the shaded sides: **cube S ≥ 150, V ≥ 30; box S ≥ 140, V ∈ [20, 220]**, hues unchanged; every object 96–100 % in range under both lights, no floor pixel in either [MES-52] (#175). One guard added: a **partly hidden cube** (its blob under 0.85 of a whole cube's silhouette at its point) is not published, like a truncated one [MES-53] (#167). `05` §2.

## Update (2026-11-06, #70)

On the real arm the ranges of `perception_real.yaml` were tuned by hand on the live debug image, not offline on recorded images. The red cube and a green bowl hold through the real cycles. The tool that measures them and the labeled real images are #223.
