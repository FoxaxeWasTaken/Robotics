# AGENTS.md

Instructions for AI coding agents. ROS 2 Jazzy pick & place for the SO-ARM101 (MuJoCo sim + real arm).
Human rules: [CONTRIBUTING.md](CONTRIBUTING.md). Design: [docs/project/](docs/project/README.md).
Work comes from GitHub issues: the issue body is the spec, its **Out of scope** section is binding.
What exists today vs what is planned: the Status table of [README.md](README.md) — check the tree before assuming a package or a topic exists.

## Build and test (inside the containers, never on the host)

```bash
cd docker && docker compose up -d control viz
docker compose exec control bash -lc "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && \
  colcon build --symlink-install --packages-select <pkg> && \
  colcon test --packages-select <pkg> --event-handlers console_direct+ && colcon test-result --verbose"
pre-commit run --all-files        # host, repo root (needs the git repo): ruff, markdownlint, yaml…
bash tools/ci/guards.sh           # host, repo root (needs the git repo): project rules, must stay green
```

## Never

- Modify `core/`, `sim/`, `ros2_ws/src/so101_description/`, `docs/SUJET.md` or the slides in `docs/` (course kit).
- Command the motors other than through `/joint_command`.
- Use ground truth (`get_object_position`, `/sim/ground_truth/*`) outside `test/` and the evaluation bench.
- Use `get_camera_extrinsics` in `so101_perception`, not even in a comment (the CI guard greps the name).
- Import `so101_sim` / `lerobot` anywhere under `so101_brain` or `so101_perception`, **tests included** (CI guard; sim-backed tests go in `so101_bringup`); same rule for the phase-2 package `so101_connect4` (ADR-037).
- Modify the provided `_connect_arm` in `driver_node.py` (ADR-032). Only exception, asked by the course: comment out its two `disable_torque()` lines once the driver commands the real arm, otherwise the arm ignores every command.
- Commit `docker/.env`, rosbags, build outputs or files over 1 MB.
- Start the stack with `USE_SIM=false` or send anything to the real arm: real-arm work is done by humans, with the checklist of `docs/project/07` §7 (tickets labeled `safety`).
- Push to `main` (PRs only), to `upstream` (the course kit) or to the school repository (overwritten by the mirror); never force-push.
- Rewrite the decision of an accepted ADR: add an "Update" section, or a new ADR that supersedes it.

## Conventions

- Conventional commits `feat(driver): …`, PR title in the same format (it becomes the squash commit); branch `<type>/<issue#>-<topic>`; types and scopes: CONTRIBUTING §2.1. PR body: one `Closes #<issue#>` line per ticket; a PR may group related tickets (same files or same roadmap step), one commit per ticket.
- Units: rad everywhere; gripper in % (0–100) only in `/joint_command`.
- Logic in pure modules with pytest tests; ROS callbacks only store messages, no `sleep` inside.
- Interface contract (topics, QoS, frames): `docs/project/01` §3. Changing it needs both owners' approval.
- A referenced doc is ambiguous? Decide, say so in the PR, update that doc in the same PR.
- A PR updates every doc it makes wrong (CONTRIBUTING §6): `git grep` the parameter, topic, file or ticket it touches in `README.md`, `docs/project/` and `results/`; fix the status tables, the "not handled" / "to add" it settles, the counts. Write what exists, in the present, never "since step X".
- Docs, commits and code comments in English.
- Every PR shows its evidence: the commands run and their output, or the measurement (CONTRIBUTING §8).
- Spikes and measurement scripts go in `docs/project/experiences/`, never in a package; a new measurement gets a `MES-xx` row in `docs/project/11` §4. New ADR or MES: update its index and the counts in `docs/project/README.md`.
- Workflows: actions pinned by commit SHA (version in a comment), `persist-credentials: false`, `${{ }}` expressions passed to `run:` through `env:` (zizmor checks it).
- Workflows: the Claude Code skills of `.claude/skills/` (`new-ticket`, `open-pr`, `measure`, `isolated-stack`, `release`, `nightly-triage`) give the order of each recurring procedure and point to the doc that holds it (CONTRIBUTING §2, §6, §8; `docs/project/11` §4, `14` §6, `15` §6, `16`); a skill never overrides this file.
- Running the stack or tests next to a developer's: `colcon test` already gives each package a free `ROS_DOMAIN_ID`, never the current one (colcon-ros-domain-id-coordinator); a second stack needs its own compose project (`docker compose -p <name>`) and `ROS_DOMAIN_ID`, or `tools/so101 run` (a throwaway container, no network, the sources copied). Never stop or restart their containers.

## Known traps

- `ros2 topic pub --once` often exits before delivery: use `--once -w 1 --keep-alive 1`.
- Camera images are `rgb8`: use `cv2.COLOR_RGB2HSV`, not BGR.
- IK seeds: `pan = −azimuth`, measured from the pan axis (x = 0.0388 m), not the origin.
- The MuJoCo `gripperframe` site is rotated 90° vs the URDF TCP: compare positions only.
- ikpy follows the **first** child joint in URDF order: assert the chain ends at `gripper_frame_joint`; clip IK seeds into the joint bounds (else scipy: "x0 is infeasible").
- `import pinocchio` needs `source /opt/ros/jazzy/setup.bash` (apt package).
- OSMesa (llvmpipe) renders with one thread per core and starves the 50 Hz control tick: the driver sets `LP_NUM_THREADS` to half the cores [MES-34]; remember it before adding another renderer.
- The provided `get_camera_intrinsics()` puts the principal point at `(W/2, H/2)` (OpenGL: pixel edges on integers); OpenCV wants pixel centers on integers: use the driver's `camera_info` (or `so101_driver.camera.opencv_intrinsics`), never the raw K, or every centroid is 0.5 px off [MES-41].
- On WSL2 the system clock steps back ~2 s every 30 s: timers on the steady clock (`create_timer(..., clock=Clock(clock_type=ClockType.STEADY_TIME))`), durations with `time.monotonic()`, stamps on the ROS clock; tests count messages, never fixed windows [MES-37].
- `pytest.importorskip` (or any skip) at a test module's level skips **every** test of the package, silently: `launch_testing`'s plugin imports each module while pytest collects the folder. Skip inside the test, and read the suite's summary line (`so101_connect4/test/test_engine_benchmark.py`).
- A package's first test needs `tests_require=["pytest"]` in `setup.py` and `<test_depend>python3-pytest</test_depend>` in `package.xml`; otherwise `colcon test` falls back to unittest and fails with exit 5 ("NO TESTS RAN").
- A MuJoCo `include` cannot change an included body (raise the base, add a camera): edit a copy with `mujoco.MjSpec`, never `sim/` (ADR-036 update).
- A CPU-bound worker thread (ikpy, scipy) delays the node's timers through the GIL: run it in a `ProcessPoolExecutor` with the `forkserver` context, never `fork` in a multithreaded node [MES-40].
- rclpy's SIGINT handler raises `KeyboardInterrupt` at any bytecode: landing while the executor builds a `Task`, it leaves a half-built one whose `__del__` prints a traceback at exit (about 1 stop in 150). The driver and perception turn rclpy's handlers off and stop their spin loop on an event (#158, #177); do the same in a node whose stop must be clean.
- rclpy fixes the severity of each logging call site: `(log.warning if bad else log.info)(text)` raises `ValueError: Logger severity cannot be changed between calls` the first time the other severity comes; write two calls.
- A 640×480 image is 0.9 MB: a best-effort subscriber (`ros2 topic hz`) gets about 1 Hz of the 10 published, a Reliable one all of them [MES-67]; subscribe Reliable, as `board_reader_node` does.
- `CONNECT4=true` (game mode, sim only): the driver serves `/sim/connect4/*` and no `/sim/randomize`, so `tools/so101 cycle` refuses, and the brain refuses `/brain/start` (the way to `PARK` crosses the grid); `down` then `up` to change mode, `restart` keeps the old one. The brain's `/brain/drop` starts from `HANDOVER` only (`not_at_handover`): call `/brain/handover` first; both reply at the end of the move. From outside, a game is `so101_connect4`'s `client.py` (`tools/so101 play`, `tools/so101 bench --game`): `/connect4/new_game` is refused until `/connect4/state` says `ready`, and a turn is `give_token`, `drop_token`, then `/connect4/human_done`; the sim calls live in `so101_bringup/sim_hand.py`, never in `so101_connect4` (guard).
- In a fresh `docker compose exec`, `ros2 node list` or `ros2 topic echo` may show nothing at first (DDS discovery): use `--no-daemon --spin-time 3`, or retry.
- "Brain node ready" does not mean the brain has `/joint_states` yet (DDS discovery): a `/brain/start` or `/go_to_target` right after it is refused (`no_joint_states`). Tests and tools wait for the first `/end_effector_pose`, which the brain computes from them (#119, #132).
- Files written in the containers (`build/`, `install/`, `log/`, `__pycache__`) belong to root: delete them from inside a container.
- `pyproject.toml` excludes course files still identical to upstream, each by name: remove a file from `extend-exclude` when you rewrite it, or it is never linted; never add a glob, it catches the team's files too (#161).
- ikpy knows no collision volumes: `Kinematics._solve` rejects the solutions `self_collides` flags (the arm folded onto its shoulder or base, `04` §8) [MES-43]; keep every IK call behind `_solve`, and still seed from a bent pose (q = 0 is singular).
- Real arm: the driver gives LeRobot one `max_relative_target` per motor, so every `send_action` must name the 6 motors or LeRobot raises a `ValueError` (#194); a ROS parameter file loaded later wins, so `driver_real.yaml` comes last (#197); LeRobot's joint zeros are not the URDF's: `joint_zero_offsets` (#196). The sim checks none of these.
- Real arm: `tools/so101 restart` is needed after a config change, and it parks the arm then cuts the torque; `disable_torque()` no longer frees the arm, the 50 Hz `send_action` turns the torque back on: stop the stack to move it by hand. `arm.json` and `joint_zero_offsets` belong to one arm: a "does not match the motors" kills the driver before it opens the camera, so RViz shows neither arm nor image.
- A real webcam's usbip link can drop, and the driver does not reopen it: stop the stack before `usbipd attach`, or it comes back as `/dev/video1` (`13`).
- With `USE_SIM=false` in `docker/.env`, tests that start perception load the real values: `USE_SIM=true tools/so101 test …`.
- More: `docs/project/13` (runbook) and `04` §8 (kinematics pitfalls).
