# ADR-013 — Own extrinsic calibration: spread multi-marker ArUco

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (own calibration) + DECIDED | 2026-10-08 | VIS → QA, KIN, CTRL | C-23 |

## Context

`get_camera_extrinsics()` is only allowed to unblock development; nobody provides that matrix on the real arm. The course demo used a single ArUco marker.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Single ArUco (demo) | Minimal setup | Ambiguous planar pose: p95 1 m / 86°, 18 mm mean object error with 0.5 px noise [MES-25] |
| **≥ 4 ArUco markers spread over the whole image, known positions, SQPNP + LM `solvePnP`** | 0.9 mm / 0.06° (no noise), 1.6 mm object error with noise [MES-16], [MES-25]; same tool in sim and real | Visible markers needed (calibration scene in sim) |
| ChArUco board | Sub-pixel corners; also serves intrinsics | Rigid board to print |
| Robot as a target (marker on the gripper / TCP touch probing, `calibrateRobotWorldHandEye`) | Directly in the robot frame | Medium effort; best as validation |
| Manual camera pose measurement | Fast | 1° rotation error ≈ 1 cm at 60 cm |

## Decision

≥ 4 (ideally 6) `DICT_4X4_50` markers of ≥ 7.5 cm, **spread over the whole image** (not only the grasp area), outside the grasp area. Calibration at **640×480** (extrinsics are resolution-independent), averaged over 30 frames. Reprojection check < 1 px (sim) / < 2 px (real). **Robot validation**: the TCP touches 4 known points, gap < 1 cm. Output `camera_extrinsics.yaml` → static TF. Sim: **`scene_calib.xml` in our repo**, including the provided scene, loaded through `scene_path`. Real: jig mat or TCP touch probing.

## Consequences

- Reprojection self-check at startup (camera moved).
- CI guard: no `get_camera_extrinsics` in the final pipeline.
- Single-marker fallback only with ≥ 15 cm, fully visible, `IPPE_SQUARE`, averaging and mandatory robot validation.

## Evidence / validation

[MES-16]: 320×240 → 23 mm; 640×480 → 0.9 mm / 0.06°. [MES-25]: methods compared with 0.5 px noise (`05` §10).

## Reconsider if

The camera becomes mobile (mounted on the arm): hand-eye calibration.

## Update (2026-10-09)

The provided `_connect_arm` builds `SO101Sim()` with the default scene: `scene_path` is applied by reconnecting right after it, without modifying it (ADR-036, same loading code as the phase-2 scene).

## Update (2026-10-16, teaching staff)

Our own calibration scene is allowed, and so are markers other than ArUco if they work better. The sim camera may be moved (`05` §6): recalibrate after any move.
