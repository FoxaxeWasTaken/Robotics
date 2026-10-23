# ADR-039 — The repository starts from the course stubs; the bootstrap prototype stays outside

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-15 | CP → ARC, QA, CTRL | C-07, C-50, C-53 |

## Context

During the bootstrap sessions (days 1–2) the team filled the course stubs with a working prototype: driver (`/joint_states` at 30 Hz, `/joint_command` with partial commands and a speed limiter, camera thread), brain (ikpy FK on `/end_effector_pose`, `/go_to_target`), an RViz file, a keyboard teleop. It was written fast, partly by trial and error, and only partly reviewed. The team repository is created on 2026-10-15.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Push the prototype as the starting code | Things work from day one | Code nobody reviewed becomes the base; most of the history is one commit; less code written by each member (C-07, C-50) |
| **Start from the course stubs, keep the prototype outside** | Every line goes through a ticket, a PR and a review; each member writes and understands a part (C-50) | The bootstrap features are rewritten |

## Decision

- The repository's `ros2_ws/` is **the course version** (upstream `42737bf`), with only a 3-line lint fix in the driver and brain stubs so that CI is green.
- The prototype is kept **outside the repository** (local copy of the member who wrote it), as a source of inspiration; it is never pushed.
- Its measurements stay valid as **feasibility evidence**: in the documents and ADRs written before 2026-10-15, "current code", "merged driver" and "implemented" refer to this prototype.
- The bootstrap features become tickets again (`09` steps 1.0, 1.2, 2.1).

## Consequences

- The driver starts again from the provided `_connect_arm` as the course ships it, `disable_torque()` lines included: they are commented out when the driver starts commanding the real arm (ADR-032).
- The README quickstart only checks the environment until the driver and the brain exist.
- `docs/project/experiences/` (design spikes, not bootstrap code) stays in the repository: it reproduces the MES measurements.

## Evidence / validation

Prototype measurements: [MES-24] (30 Hz, 0.13 mm through `/go_to_target`); real-arm tests 1–3 of `07` §3.2 passed with it (2026-10-09). The course stubs build and start ("Driver node ready.", "Brain node ready.") and pass CI (lint, guards, build) [CODE].

## Reconsider if

Time runs short before the mid-term review: port one prototype block at a time through normal PRs, with tests, instead of rewriting it.

## Update (2026-10-24, #134)

- **The prototype gets no link either**: it was never published and stays a local copy. The README says it stays outside this repository and links only the course repository (`upstream`), where the kit comes from (team decision, 2026-10-24).
- The README quickstart now runs a full cycle (`tools/so101 cycle`): the driver, the perception and the brain exist since steps 1 to 4.
