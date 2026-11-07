# P2-04 — Phase-2 roadmap (Connect Four)

> Author: [CP] Project lead · Reviewers: all experts
> Same rules as the phase-1 roadmap [`../09_roadmap_step_by_step.md`](../09_roadmap_step_by_step.md), which this document details for its stage 7: every step ends with **evidence** ("Done when"), the riskiest part first. **Unlike `09`, this plan has dates** (§3): the hand-in is 2026-11-20 23:42. Lanes as in `09` §1.2: **A** driver, infra and hardware · **B** kinematics and brain · **C** perception.
> Tickets: `P2.<step><letter>`, under the stage-7 epic (`../15` §12) · Decisions: ADR-034 to ADR-038 · Risks and tests: [`P2-03`](P2-03_plan_risks_tests.md) · Targets `P2-xx`: [`P2-01`](P2-01_scope_and_demo.md) §4

## 1. Overview

`v2.0.0` ends at P2.6. The rest is `v2.1.0`, after the hand-in.

| Step | Lane | Tickets | Outcome | Level (`P2-01` §2) |
| --- | --- | --- | --- | --- |
| **P2.0** Pitch ✅ | CP | #73, #74 | Validated by the teaching staff; scope set to the sim | — |
| **P2.1** Layout and drop spike | B | #75, #76 | One layout, go / no-go on dropping | Go: 1; no-go: 0 |
| **P2.2** Game engine | B | #77, #78, #230 | Package, `game.py`, `turns.py` tested | — |
| **P2.3** Scene and loading | A | #79, #80, #231 | Game scene, sim services, `CONNECT4` mode | — |
| **P2.4** Board reader | C | #81, #82 | Grid read from the camera | — |
| **P2.5** Token in the gripper and `/brain/drop` | B | #83, #84, #85 | One move through the brain, `fsm.py` untouched | — |
| **P2.6** Sim integration | Shared | #86, #87, #232, #233, #234 | Full games in sim, a console, the CI | **1** |
| **P2.7** Real arm (`v2.1.0`) | Shared, A leads | #88 to #91 | Full games on the real arm, filmed | **2** |
| **P2.8** Wrist camera (`v2.1.0`) | A, C, B | #92 to #95 | Look and correct, if kept (`P2-02` §8) | 2, more reliable |
| **P2.9** Stretch | Anyone | none | One or two bonus features, only if ahead | **3** |

### 1.1 Dependencies

```mermaid
flowchart LR
  S1["P2.1 Spike<br/>#76"] --> S3["P2.3 Scene, services, mode<br/>#79 #80 #231"]
  S2["P2.2 Package, engine, turns<br/>#77 #78 #230"] --> S3
  S3 --> S4["P2.4 Reader<br/>#81 #82"]
  S2 --> S4
  S3 --> S5["P2.5 Drop<br/>#83 #84 #85"]
  S2 --> S6["P2.6 Game<br/>#86"]
  S4 --> S6
  S5 --> S6
  S6 --> B["Bench, console<br/>#87 #232"]
  B --> C["CI, guide, video<br/>#233 #234 #97"]
  C --> R["Release<br/>#229"]
  S1 -. "no-go" .-> F["Level 0<br/>tic-tac-toe with cubes"]
```

**Critical path**: #76 → #79 → #80 → #84 → #86 → #87 → #229. #75 blocks nothing: the sim runs on the model of `P2-02` §7, and every dimension is a parameter.

### 1.2 Who does what

| Lane | Tickets | Note |
| --- | --- | --- |
| A — Driver, infra | #79, #80, #231, #233 | Scene, sim services, mode, CI |
| B — Kinematics and brain | #75, #76, #77, #78, #83, #84, #85, #230 | Spike, engine, turns, `/brain/drop` |
| C — Perception | #81, #82 | Grid reading |
| Shared | #86, #87, #232, #234, #97, #98, #229 | Everyone plays a game and can explain each block before the defense (C-50) |

---

### Step P2.0 — Pitch validated ✅ (7.1a, 7.1b)

- **Done**: validated by the teaching staff on 2026-10-09 (ADR-034); questions of `P2-03` §4 answered on 2026-10-16; scope set to the sim on 2026-11-07, after an expert review that measured it [MES-63 to MES-66].

### Step P2.1 — Layout and drop spike (#75, #76)

- **Goal**: know that the arm can drop a token, on one layout, before building around it. The expert review proved each part on a prototype: #76 puts them together.
- **Tasks**: `experiences/connect4/scene/exp_connect4_scene.py` on the layout of `P2-02` §7 with the pose of `experiences/connect4/drop/`; 100 drops, 100 with a TCP offset of ±5 mm, capture sweep, full stacks read with the method of `experiences/connect4/reading/`. The purchased grid's numbers (#75) go in if they are in hand.
- **Done when**: results recorded; **go** if ≥ 95 % without TCP offset and ≥ 80 % with ±5 mm. No-go: change the sim geometry (slot gap, guide), it is a parameter; still no-go: **level 0** with the same software.

### Step P2.2 — Game engine (#77, #78, #230)

- **Tasks**: the `so101_connect4` package with the guards extended to it; `game.py` and `turns.py`, ported from `experiences/connect4/game/` with their tests.
- **Done when**: P2-04 met in CI (legal moves only, 100/100 against random, < 2 s per move); every transition of `P2-02` §5 tested; the arm moves in `MOVE` only.

### Step P2.3 — Scene and loading (#79, #80, #231)

- **Tasks**
  - [ ] Scene generated with `MjSpec` from a copy of the provided one: floor and table lowered, the base at the origin, grid, depth guide, 42 tokens, ArUco markers, the working set (`P2-02` §7).
  - [ ] Sim services `/sim/connect4/drop_token`, `give_token`, `reset_board`, the true board under `expose_ground_truth`.
  - [ ] `CONNECT4` from `docker/.env` to the launch files (`P2-02` §6.1).
- **Done when**: tick p99 ≤ 25 ms and image ≥ 3 Hz with 42 tokens; a `drop_token` lands in the right cell and a `give_token` then a close holds the token; with the mode off, the generated scene is identical and the E2E job is green.

### Step P2.4 — Board reader (#81, #82)

- **Tasks**: `board.py` and `board_reader_node` (`P2-02` §3): grid pose from the ArUco markers, 42 cells classified, stable reads only; debug view in RViz.
- **Done when**: P2-01 met in sim (50 states, 100 % of the cells); grid pose within 5 mm of the scene (test only).

### Step P2.5 — Token in the gripper and `/brain/drop` (#83, #84, #85)

- **Tasks**
  - [ ] `release_pose` in `so101_brain/drop.py`: roll +1.6195 rad, tilt 70°, target = token centre.
  - [ ] `/brain/drop` (reply at the end of the move), `/brain/handover`, the held-token check; `fsm.py` and `grasp_planner.py` unchanged.
  - [ ] Interface contract updated in `01` §3 with both owners' approval.
- **Done when**: 50 sim drops ≥ 95 % (P2-02), token held ≥ 49/50 (P2-03); 10 tries without a token refused, the arm never moves; the phase-1 nightly unchanged.

### Step P2.6 — Sim integration (#86, #87, #232, #233, #234): level 1

- **Tasks**: `game_node`, the console `tools/so101 play`, the game bench, phase 2 in the CI, the player guide and the results file.
- **Done when**: P2-06 (≥ 8/10 full games), P2-07 (a member who did not write the game plays from a fresh clone), P2-08 (a game on each PR, the games in the nightly report).

### Step P2.7 — Real arm (`v2.1.0`, #88 to #91): level 2

Riser, jig, depth guide, commissioning again on the riser (`07` §3), handover tuning, grid webcam, real benches, then games against people. **Read `P2-02` §8 first**: five findings block any session next to a person, and the first task is a real spike on 7 poses.

### Step P2.8 — Wrist camera (`v2.1.0`, #92 to #95)

Look and correct before the release, compared with the guide on the real arm. The sim arm has no gear backlash, so a sim result says nothing here: only the real comparison decides. The 2026-11-07 review proposes to cut it (`P2-02` §8).

### Step P2.9 — Stretch (optional): level 3

One or two features of `P2-02` §9, only once P2.6 is green and ahead of §3. Difficulty levels are a parameter of #78.

## 2. Delivery

Phase 2 ends in `09` stage 8: the sim video, the team on camera, the slides (the pitch, the edge-on geometry [MES-27], the bench numbers, and why the sim), tag `v2.0.0`. `v2.1.0` brings the real arm.

## 3. Calendar to the hand-in

| Date | Gate | Tickets |
| --- | --- | --- |
| Sat 7 – Sun 8 | Scope docs merged | #74 |
| Mon 9 | Spike go / no-go; package, engine, turns, drop pose | #76, #77, #78, #230, #83 |
| Tue 10 – Thu 12 | Scene, services, mode (the 11th is a holiday) | #79, #80, #231 |
| Thu 12 – Sat 14 | Reader, `/brain/drop`, handover | #81, #82, #84, #85 |
| **Sat 14** | **First full game on the stack** | #86 |
| Sun 15 | Console; game bench; a game on each PR | #232, #87, #233 |
| Mon 16 – Tue 17 | Games in the nightly, guide, results. **Tue 17: target for the last feature PR**, nightly dispatched at once | #233, #234 |
| Wed 18 | Videos, the team on camera. 22:00: latest safe feature PR | #97 |
| Thu 19 | Rehearsal. 12:00: latest safe last PR, with `[2.0.0]` and the docs, opened once the gate is read. 22:00: safe tag | #98, #229 |
| Fri 20 | 12:00: latest tag. The tag on the school copy checked. **23:42: hand-in** | #229 |

The worst chain from the last feature merge is about 35 h (nightly, a fix with a 15 h review, nightly, last PR with a 15 h review, nightly: estimated), hence the 2026-11-17 target. Tickets grouped in PRs by this table, one commit per ticket.

## 4. Cut-offs

| Date | If | Then |
| --- | --- | --- |
| 2026-11-09 | Spike no-go | Change the sim geometry; level 0 only as a last resort |
| 2026-11-10 | Contact unstable in the merged scene | The token welded to the jaw, sim only |
| 2026-11-11 | Tokens do not stack | `drop_token` places them at their rest poses |
| 2026-11-12 | Token hold through ROS unproven | A sim-only weld on the pinch |
| 2026-11-12 | No grid measured | #75 moves to `v2.1.0`, the sim stays on the model |
| 2026-11-14 | No full game on the stack | Cut the polish of the console and the engine human of the bench |
| 2026-11-14 | Drops below 95 % because of the pose | Targets from `connect4_grid.yaml`, markers kept as a check |
| 2026-11-16 | Bench not stable | 3 games in the report instead of 10, said in the results |
| 2026-11-18 | A ticket not merged | Moved to `v2.1.0`: `release.sh` refuses an open ticket |
