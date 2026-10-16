# ADR-032 — Real-arm connection: keep the provided `_connect_arm`

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (revised after the upstream pull) | 2026-10-08 | CTRL/SAFE → CP, OPS | C-10, C-51 |

## Context

The 2026-10-08 pull (upstream PR #1 `fix/connect_arm`, commit `bc00df7`) added a **provided** `_connect_arm` to the driver stub. The team's local version and the first version of `03` §2 used another policy on calibration mismatch: **write** the file's calibration into the motors.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Keep the local version (`write_calibration` on mismatch) | The arm always starts | Writes offsets **without human check**: a file from another arm produces wrong motions; diverges from the course code (conflicts on every sync) |
| **Adopt the provided version** (`disconnect` + `RuntimeError` on mismatch) | Safe (C-51): a human checks; no divergence from upstream | Manual recalibration or file fix needed |
| Merge both (`--force-write-calibration` option) | Flexible | Complexity, and a dangerous parameter at hand |

## Decision

**Keep `_connect_arm` exactly as provided.** The team's additions live **next to it**: `max_relative_target` safety net set after the call (ADR-023), publishers/timers and execution model (ADR-006). The merge was done on 2026-10-08: the diff with upstream only fills the `TODO`s.

## Consequences

- Provided error message: "Re-run lerobot-calibrate on the host" — the image has no `lerobot-calibrate` CLI; the recalibration procedure is in `07` §3.1.
- New runbook row (`13`) for that message.
- Minimal diff with upstream: future syncs stay simple (ADR-027).

## Evidence / validation

`git show bc00df7`; review RC-29; merged driver verified: 6/6 tests, 30 Hz, clean shutdown, `/go_to_target` 0.13 mm [MES-24].

## Reconsider if

The teaching staff explicitly recommends automatic calibration writing (e.g. for replaced motors).

## Update (2026-10-09, upstream PR #2, commit `42737bf`)

The provided `_connect_arm` now ends with `robot.bus.disable_torque()` and a "Torque disabled" log, with the comment "Comment out this line to control the robot". Following that instruction, the team keeps these lines **commented out**: it is the **only** change to the provided function. Uncomment them for a session where the arm must be moved by hand (`07` §3.2 test 1, taught poses of `phase2/P2-02` §2.4). `destroy_node` is provided too (disconnect + log); the team stops the camera thread before it (`03` §8). The diff with upstream stays minimal.

## Update (2026-10-15, ADR-039)

The repository starts from the course stubs, so `_connect_arm` is exactly as shipped, `disable_torque()` lines **active**. The exception above still holds: comment those two lines out in the PR where the driver starts commanding the real arm (step 1.0 or 6.1), and keep them active only for hand-moving sessions.
