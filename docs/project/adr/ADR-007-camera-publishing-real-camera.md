# ADR-007 — Camera: publication and real camera

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (image ~10 Hz) + DECIDED | 2026-10-08 | VIS → CTRL, OPS | C-12, C-13 |

## Context

The driver must publish the image (~10 Hz) and the intrinsics. On the real arm the USB camera goes through usbip under WSL2.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Camera inside `SO101FollowerConfig.cameras` | LeRobot API | Camera reads happen inside `get_observation()`, i.e. in the control loop |
| **Separate `OpenCVCamera` in its own thread** | Control and vision decoupled | One more object in the driver |
| Default format (YUYV) | — | 0 fps through usbip [MES-17] |
| **MJPG** | 27.5 fps at 640×480 through usbip | CPU decoding (negligible) |

## Decision

The driver publishes `Image` (`rgb8`) + `CameraInfo` **with the same `stamp`**, `frame_id = external_cam_optical_frame`, at 10 Hz. K: `get_camera_intrinsics()` in sim, calibration YAML on the real arm. Real camera: `OpenCVCamera(fourcc="MJPG")` **outside** `SO101Follower`.

## Consequences

- Add `camera_info` to the current code (missing).
- Rescale K if the resolution changes (calibration at 640×480).

## Evidence / validation

[MES-17]; code `cameras/opencv/configuration_opencv.py` (`fourcc`).

## Reconsider if

A depth camera (RealSense) is added: also publish `depth/image_raw` and revisit ADR-012.
