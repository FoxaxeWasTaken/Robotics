# ADR-004 — Interface contract and QoS

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| PARTLY IMPOSED | 2026-10-08 | ARC → VIS, DEC, CTRL | C-11 to C-14, C-21, C-34, C-35 |

## Context

Some names/types are imposed (`/joint_states`, `/joint_command` as JointState, `/external_cam/image_raw`, `/brain/start|stop`), others suggested (`/object_position`, `/drop_box_position`). A QoS mismatch makes a topic silent **without any error**.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| `PointStamped` for positions | Minimal | No yaw; incompatible with `drop_box_marker_pub` (provided, expects `PoseStamped`) |
| **`PoseStamped`** | Carries the cube yaw; compatible with the provided marker | Useless orientation for the box |
| `vision_msgs/Detection3D` | Standard, carries confidence | Package missing from the image; overkill |
| Images published Best effort | Typical for sensor streams | Reliable subscribers (RViz, `echo` by default) → 0 message |
| **Images published Reliable**, free subscribers | Compatible with every subscriber | Slightly more traffic (negligible: 230 KB at 10 Hz) |

## Decision

Contract of `01` §3: `/object_position` and `/drop_box_position` as **`PoseStamped`** in `base_link`, published **only when the detection is valid**; **every publisher is Reliable** (except debug); perception subscribes to images Best effort, depth 1; `/brain_state` is `transient_local`.

## Consequences

- Any contract change goes through a PR reviewed by both sides.
- The brain judges freshness through `header.stamp` (ADR-017).

## Evidence / validation

QoS compatibility rule [KO1 p29]; provided marker (`drop_box_marker_pub.py`).

## Reconsider if

Rich detection messages are adopted (confidence, truncation): create a `.msg` in `so101_interfaces`.
