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
