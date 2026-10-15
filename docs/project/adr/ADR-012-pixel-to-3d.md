# ADR-012 — Pixel → 3D: ray and table plane

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (own assumption) + DECIDED | 2026-10-08 | VIS → KIN | C-22 |

## Context

The camera is monocular: an assumption is needed to get depth.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| **Object resting on the table**: ray / plane z = 2.5 cm intersection | Simple, exact for a resting object; 2.7 mm error | Wrong if the object is not resting (in hand, stacked) |
| Known apparent size | No table assumption | Inaccurate at 320×240 (~15 px per side) |
| Depth (RGB-D) | No assumption | Not available in sim |

## Decision

Ray `K⁻¹[u, v, 1]` → rotation to `base_link` → intersection with **z = 0.025 m** (cube center); same for the box (z = 0.025, floor center published at z = 0). Yaw from `minAreaRect` on the contour back-projected at z = 0.05. Optional constant bias correction; median filter over 5.

## Consequences

- The table defines z = 0 (base resting on it).
- Real arm: `undistortPoints` before back-projection.

## Evidence / validation

[MES-13]: 2.7 mm mean (0.8 mm after bias correction); [MES-14]: yaw 1.3° median.

## Reconsider if

Stacking or grasping non-resting objects (phase 2): known size or RGB-D.
