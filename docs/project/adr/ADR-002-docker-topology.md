# ADR-002 — Docker topology: control, perception, viz

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (2 containers) + DECIDED (additions) | 2026-10-08 | OPS → ARC, VIS | C-02, C-04 |

## Context

The brief imposes two containers, `control` and `perception`, on a shared ROS 2 network. The provided compose file has **no** `perception` service although the slides use it, and forbids concurrent `colcon build` on the same volume.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Add `perception` with a separate build (`build_perception/`, `install_perception/`, `--packages-up-to`) | Compliant; no build conflict; light image | Two install spaces to source |
| `perception` waits for `control`'s build and sources its `install/` | Single build | Coupling; the perception image lacks control's dependencies |
| Everything in `control` | Simple | **Violates C-02** |

## Decision

Add a **`perception`** service (image `tek5/perception`) building **its own packages into separate folders**. Keep `viz` (RSP + static TF + marker) as is. Bridge network `rosnet`, `ROS_DOMAIN_ID` in `.env`.

## Consequences

- `build_perception/` and `install_perception/` are ignored by the root `.gitignore`.
- Perception Python dependencies go into `Dockerfile.perception` (dedicated layer, pinned versions).
- The real camera is plugged into `control` (the driver publishes it; cgroup rule `c 81:*`).

## Evidence / validation

Inter-container communication checked with `talker` and a `tek5/perception` container on `rosnet` [MES-18].

## Reconsider if

The teaching staff publishes an official compose file with `perception`: align with it.

## Update (2026-10-16, teaching staff)

The staff confirms the `perception` service is expected (the Dockerfile is provided, the service was missing from the compose file) and that the Docker files and the nodes may be adapted freely.

## Update (2026-10-30, #189)

The stack keeps perception in its own container (C-02). Two test tools start a stack of their own instead, in the `control` container and on their own ROS domain: the evaluation bench (`sim.launch.py expose_ground_truth:=true perception:=true`, domain 77, `08` §3) and the robustness demo (`robustness_demo.launch.py`, `results/robustness_demo.md`). They never share a domain with the everyday stack.

## Update (2026-11-14, #274, #273)

**DDS over UDP only**: `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, set in both images and as the compose file's default (it was `LARGE_DATA` there, and `DEFAULT` in the E2E and nightly jobs since #141).

Fast DDS's shared-memory transport is part of `DEFAULT` and of `LARGE_DATA`. With it, a process that joins the ROS graph (a rosbag recorder, RViz, a `ros2` command) makes a `publish()` of the nodes already there block for about 0.3 s, and rclpy publishes with Python's GIL held: every Python thread of the driver stops, so its 50 Hz control loop. Over 0.5 s the brain ends in `ERROR:no_joint_states`; it cost the nightly bench an episode twice. Over 150 recorders started on an idle stack: 11 holds with `DEFAULT`, 10 with `LARGE_DATA`, none with `UDPv4` [MES-80].

Between two containers the stack already spoke UDP under `DEFAULT` (shared memory does not cross them): the camera's images reach perception that way in every E2E job and nightly since #141. What changes is inside a container, and the everyday stack, which leaves `LARGE_DATA`'s TCP.

Reopen if the images are late or lost over UDP on a machine (a `ros2 topic hz` of `/external_cam/image_raw` under 10 Hz with a Reliable subscriber): then shared memory for the camera's topic only, by a Fast DDS profile.
