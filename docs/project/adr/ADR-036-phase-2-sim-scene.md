# ADR-036 — Phase-2 sim scene: our scene file, loaded next to the provided `_connect_arm`

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-09 | CTRL → ARC, VIS, QA | C-05, C-06, C-10 |

## Context

Phase 2 needs a scene with a grid, a funnel, a magazine, tokens, ArUco markers and a camera facing the grid. `sim/` must never be modified (AGENTS.md, R-16), and the provided `_connect_arm` builds `SO101Sim()` with the default scene (ADR-032). Phase 1 has the same need for its calibration scene (ADR-013, `05` §5).

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Edit `scene_tek5.xml` in `sim/` | Simple | Forbidden; lost at the next upstream update |
| Pass `scene_path` inside `_connect_arm` | One line | Modifies the provided function (ADR-032) |
| **Our scene file including `scene_tek5.xml`, reloaded right after `_connect_arm`** | Provided files untouched; same mechanism as the calibration scene | A temporary folder: MuJoCo resolves `include` paths relative to the main file |
| A full copy of `scene_tek5.xml` in our repository | No include path issue | Silent drift from upstream |

## Decision

- `scene_connect4.xml` lives in our repository and **includes** `scene_tek5.xml`. At startup, the provided asset folder is copied (or linked) into a temporary folder next to our scene file.
- In the driver, **after** the provided `_connect_arm` returns, in sim and only if the `scene_path` parameter is set: `disconnect()`, set `scene_path` and `camera_name` (`board_cam`), then `connect()`. `SO101Sim.connect()` reloads the model from `scene_path` [CODE].
- The phase-1 cube stays in the scene, because `connect()` calls `randomize_object()`, which needs `pickup_object_free`. It is moved out of the play area right after loading.
- The simulated human is a sim-only driver service, `/sim/connect4/drop_token` (column, color). It acts on the scene and returns no ground truth; the pipeline still reads the grid through the camera (C-06).

## Consequences

- The same loading code serves the phase-1 calibration scene and the phase-2 scene.
- Contact parameters of thin tokens in narrow slots must be tuned (spike P2.1).

## Evidence / validation

`SO101Sim` API read in `sim/so101_sim/__init__.py` [CODE]; include-based scene validated by `exp_perception_calib.py` (phase 1); camera need [MES-31]. To validate: integration test of the loading (`phase2/P2-04` step P2.3).

## Reconsider if

The course adds a `scene_path` argument to the provided connection, or `SO101Sim.connect()` stops reloading the scene.

## Update (2026-10-09, ADR-038)

The arm sits on a riser and gets a wrist camera, two changes inside the provided robot model, which an `include` cannot make. The phase-2 scene is therefore **generated** from a copy of the provided scene with `mujoco.MjSpec` (`spec.body("base").pos[2] += riser`, `spec.body("gripper").add_camera(...)`, then our objects), written with `to_xml()` into the temporary folder and loaded as before. Checked with MuJoCo 3.15: the model compiles and the wrist camera renders [CODE]. The provided files stay untouched.

## Update (2026-10-16, teaching staff)

The staff allows changing the scene: our own scenes (calibration markers, phase-2 grid) and the pose and parameters of the scene camera, for instance to match the real camera. The rule stays: edit a copy with `mujoco.MjSpec`, never `sim/`.

## Update (2026-10-26, #44, #47)

The mechanism exists for phase 1: `so101_bringup.sim_scene` copies the provided scene folder and edits it with `mujoco.MjSpec`; `sim.launch.py` generates the scene at launch and passes it to the driver's `scene_path`, which reconnects `SO101Sim` right after the provided `_connect_arm`. Every sim run uses it now: the external camera moved (`config/sim_scene.yaml`, #47); `calibration:=true` adds the ArUco markers (`config/markers_sim.yaml`, #44). The phase-2 scene extends the same generator. A camera of the provided scene is oriented by `xyaxes`: the copy sets `xyaxes` too, since a `quat` set alone does not override it.
