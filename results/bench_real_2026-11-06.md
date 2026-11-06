# Phase-1 real pick & place (step 6.5a, #72, C-43), 2026-11-06

The real SO-101, a USB webcam, the team's driver, brain and perception, on the setup of [`07` §9](../docs/project/07_real_arm_safety_sim2real.md). Each trial is one `tools/so101 cycle` with the cube and the drop target moved by hand.

- **Arm**: the bench's arm (servo board 5A46084770), not the one of 2026-10-30: recalibrated, its joint zeros measured again (`driver_real.yaml`).
- **Camera**: UGREEN webcam, 640×480 MJPG, 3.4 to 4.3 frames/s through usbip [MES-61], intrinsics RMS 0.36 px [MES-58].
- **Camera pose**: the ChArUco board touched by the arm at 4 corners, within 2.7 mm [MES-59].
- **Object and target**: the 5 cm cube of the course, a green translucent bowl 5 cm high.
- **Speeds**: the sim's, 1.0 rad/s in the brain and 1.5 rad/s at the limiter (ADR-023 Update).

## Result

**14/17 over the two last series, target ≥ 7/10 met** (`07` §3.2 test 6, C-43): 7/8, then 7/9 on the final code, filmed. A cycle counts when the cube ends in the bowl, retries included: the brain's own `max_retries` is part of the pick & place.

### Series of 8 trials

| Trial | Outcome | What happened |
| --- | --- | --- |
| 1 | ✅ | |
| 2 | ✅ | |
| 3 | ❌ | Dropped on the bowl's rim, the cube ended beside it: the bowl was at the far left of the reach |
| 4 | ✅ | |
| 5 | ✅ | |
| 6 | ✅ | At the third attempt: cube at 29 cm of the pan axis, the jaws closed empty (`GRASP → RETRY`, 2.4 %), which pulled it nearer; second attempt dropped on the rim, `VERIFY` saw it on the table and retried; third one good |
| 7 | ✅ | |
| 8 | ✅ | |

The series stopped there: the camera's USB link dropped (`13`, a third time that day).

### Final run, every fix in

The stack restarted on the final code, then 9 `tools/so101 cycle`, read from the brain's log: a ✅ is a `DONE`, and the two films show the cube in the bowl.

| Cycle | Outcome | What the brain logged |
| --- | --- | --- |
| 1 | ✅ | First attempt |
| 2 | ✅ | Second attempt: the first closed empty (`GRASP → RETRY`, 1.6 %) |
| 3 | ✅ | Third attempt: twice `VERIFY → RETRY`, the cube seen on the table after the drop |
| 4 | ✅ | Second attempt: the first closed empty |
| 5 | ✅ | First attempt, [filmed](https://github.com/FoxaxeWasTaken/Robotics/releases/download/v1.3.0/real_pick_and_place.mp4) |
| 6 | ❌ | `ERROR:grasp_failed`: three attempts closed empty |
| 7 | ❌ | `ERROR:object_lost`: the cube last seen at y = 59 cm, out of reach |
| 8 | ✅ | Second attempt, [filmed](https://github.com/FoxaxeWasTaken/Robotics/releases/download/v1.3.0/real_cube_moved_retry.mp4): the cube moved by hand during the approach, the jaws close empty, `RETRY` finds it again |
| 9 | ✅ | First attempt |

Cycles 6 and 7 came just before the film of cycle 8. They count as failures here, even if the cube was being moved by hand.

## Before it (not counted: the fixes they led to are in)

| Series | Result | Cause | Fix |
| --- | --- | --- | --- |
| First cycles, 0.5 rad/s | Grasp judged empty twice, the cube held | The jaws stop at 4.5 % in a cycle, 5.8 % by hand: the 5 % threshold was too high [MES-60] | `close_pct` 1 %, held from 2.8 % |
| 9 trials at the sim's speeds | 6/9 | One grasp on a corner of the cube; two `ERROR:timeout:APPROACH` with the arm in place: `wrist_roll` stopped 0.17 rad short | `reach_tol_roll_rad` 0.25 rad |

## Limits seen

- **Reach**: reliable between 18 and 28 cm of the pan axis (x = 3.88 cm). At 29 cm and beyond the grasp misses or the planner answers `unreachable`; the drop target under 14 cm gives `drop_unreachable`.
- **Rim**: a 5 cm cube held by a wall sits off the TCP, and a bowl leaves about 4 cm of margin: the drops that failed were at the edge of the working area.
- **`VERIFY` and the bowl**: through the green plastic the cube is often not seen: `DONE` with the warning "cube not seen", as in sim when it is hidden.
- **Motion**: the arm moves by small jerks. The bus is not the cause (no gap above 44 ms over a whole cycle): the servos' low gain (P = 16). Faster motions show it less.

## Reproduce

`07` §9, then `tools/so101 cycle`. The films are assets of the release `v1.3.0`, silent, not in Git (`15` §7).
