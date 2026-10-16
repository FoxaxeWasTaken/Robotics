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

- Conventional commits `feat(driver): …`, PR title in the same format (it becomes the squash commit); branch `<type>/<issue#>-<topic>`; types and scopes: CONTRIBUTING §2.1. PR body `Closes #<issue#>`; one PR per ticket.
- Units: rad everywhere; gripper in % (0–100) only in `/joint_command`.
- Logic in pure modules with pytest tests; ROS callbacks only store messages, no `sleep` inside.
- Interface contract (topics, QoS, frames): `docs/project/01` §3. Changing it needs both owners' approval.
- A referenced doc is ambiguous? Decide, say so in the PR, update that doc in the same PR.
- Docs, commits and code comments in English.
- Every PR shows its evidence: the commands run and their output, or the measurement (CONTRIBUTING §8).
- Spikes and measurement scripts go in `docs/project/experiences/`, never in a package; a new measurement gets a `MES-xx` row in `docs/project/11` §4. New ADR or MES: update its index and the counts in `docs/project/README.md`.
- Workflows: actions pinned by commit SHA (version in a comment), `persist-credentials: false`, `${{ }}` expressions passed to `run:` through `env:` (zizmor checks it).
- Running the stack or tests next to a developer's: `colcon test` already gives each package a free `ROS_DOMAIN_ID`, never the current one (colcon-ros-domain-id-coordinator); a second stack needs its own compose project (`docker compose -p <name>`) and `ROS_DOMAIN_ID`. Never stop or restart their containers.

## Known traps

- `ros2 topic pub --once` often exits before delivery: use `--once -w 1 --keep-alive 1`.
- Camera images are `rgb8`: use `cv2.COLOR_RGB2HSV`, not BGR.
- IK seeds: `pan = −azimuth`, measured from the pan axis (x = 0.0388 m), not the origin.
- The MuJoCo `gripperframe` site is rotated 90° vs the URDF TCP: compare positions only.
- ikpy follows the **first** child joint in URDF order: assert the chain ends at `gripper_frame_joint`; clip IK seeds into the joint bounds (else scipy: "x0 is infeasible").
- `import pinocchio` needs `source /opt/ros/jazzy/setup.bash` (apt package).
- OSMesa (llvmpipe) renders with one thread per core and starves the 50 Hz control tick: the driver sets `LP_NUM_THREADS` to half the cores [MES-34]; remember it before adding another renderer.
- A package's first test needs `tests_require=["pytest"]` in `setup.py` and `<test_depend>python3-pytest</test_depend>` in `package.xml`; otherwise `colcon test` falls back to unittest and fails with exit 5 ("NO TESTS RAN").
- A MuJoCo `include` cannot change an included body (raise the base, add a camera): edit a copy with `mujoco.MjSpec`, never `sim/` (ADR-036 update).
- In a fresh `docker compose exec`, `ros2 node list` or `ros2 topic echo` may show nothing at first (DDS discovery): use `--no-daemon --spin-time 3`, or retry.
- Files written in the containers (`build/`, `install/`, `log/`, `__pycache__`) belong to root: delete them from inside a container.
- `pyproject.toml` excludes course files still identical to upstream: remove a file from `extend-exclude` when you rewrite it, or it is never linted.
- More: `docs/project/13` (runbook) and `04` §8 (kinematics pitfalls).
