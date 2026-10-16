# ADR-014 — Intrinsic calibration of the real camera

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | VIS → CTRL | C-13, C-43 |

## Context

The sim provides K without distortion; a real webcam has an unknown focal length and distortion.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| **ChArUco or checkerboard + `cv2.calibrateCamera`** | Standard, accurate (RMS < 0.5 px) | Board to print |
| ROS `camera_calibration` | GUI, direct `camera_info` YAML | Package to add to the image |
| Manufacturer values | Fast | Inaccurate, no distortion |

## Decision

**ChArUco** board (or checkerboard), 20–30 varied views, `calibrateCamera`, **RMS < 0.5 px**, `camera_info` YAML output loaded by the driver on the real arm.

## Consequences

- Redo if focus or resolution changes (keep autofocus off).

## Evidence / validation

Not measured yet (real hardware pending); standard OpenCV procedure.

## Reconsider if

The chosen camera provides a reliable factory calibration (e.g. RealSense).
