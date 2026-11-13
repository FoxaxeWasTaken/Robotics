# ADR-023 — Layered safety

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (rules) + DECIDED | 2026-10-08 | CTRL/SAFE → CP, DEC | C-15, C-51 |

## Context

The real arm can break or injure. Torque off = the arm falls. A manual command can jump a joint by several rad.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Rely on human caution | — | Insufficient |
| **Defense in depth**: procedure + brain + driver + LeRobot + servos + stop + physical cut-off | No single point of failure | Settings to maintain |

## Decision

Layers of `07` §2: `v_max` in the brain; **clamp + speed limiter in the driver**; **per-motor** `max_relative_target` (a scalar would freeze the gripper in %); servo limits (configure); `/brain/stop`; **PARK before `disconnect()`**; checklist.

## Consequences

- Current code to complete: clamp (C-15) and PARK before cutting torque.

## Evidence / validation

Reading `so_follower.py` and `config_so_follower.py` (RC-12, RC-13).

## Reconsider if

Close human collaboration (phase 2): lower speeds further, limit forces, define a keep-out zone.

## Update

The course now provides `_connect_arm` (upstream PR #1) without `max_relative_target`. The provided function is **not** modified: the safety net is set right after the call (`self._robot.config.max_relative_target = {...}`, re-read on every `send_action`). See `03` §2 and ADR-032.

## Update (2026-11-06)

The team runs the real arm at the sim's speeds: 1.0 rad/s in the brain, 1.5 rad/s at the driver's limiter, instead of 0.5 and 0.8. The first real cycles at the reduced speeds were jerky (stick-slip, MES-62) and the 10 trials and the video come at the end of phase 1. The other layers stay: clamp, `max_relative_target`, slow shutdown (0.5 rad/s), `/brain/stop`, the power switch, hands off. Faster means less time to react, so the checklist of `07` §7 says it. Reconsider for phase 2, with humans near the arm.

## Update (2026-11-13, phase 2)

In game mode the driver's `shutdown_pose` is `HANDOVER`, not `PARK`: the joint line from `HANDOVER` to `PARK` takes the gripper 30 mm into the grid's near post [MES-69]. The two legs back of a move, jaws empty, go 1.4 times the brain's speed, 1.4 rad/s on the sim, under the limiter's 1.5 [MES-73]: a ratio, so an overlay that slows the real arm slows them too (ADR-040). Sim only: the real arm's speeds near a person are `v2.1.0`'s (`phase2/P2-02` §8).
