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

## Conventions

- Conventional commits `feat(driver): …`, PR title in the same format (it becomes the squash commit); branch `<type>/<issue#>-<topic>`; types and scopes: CONTRIBUTING §2.1. PR body `Closes #<issue#>`; one PR per ticket.
- Units: rad everywhere; gripper in % (0–100) only in `/joint_command`.
- Logic in pure modules with pytest tests; ROS callbacks only store messages, no `sleep` inside.
- Interface contract (topics, QoS, frames): `docs/project/01` §3. Changing it needs both owners' approval.
- A referenced doc is ambiguous? Decide, say so in the PR, update that doc in the same PR.
- Docs, commits and code comments in English.

## Known traps

- `ros2 topic pub --once` often exits before delivery: use `--once -w 1 --keep-alive 1`.
- Camera images are `rgb8`: use `cv2.COLOR_RGB2HSV`, not BGR.
- IK seeds: `pan = −azimuth`, measured from the pan axis (x = 0.0388 m), not the origin.
- The MuJoCo `gripperframe` site is rotated 90° vs the URDF TCP: compare positions only.
- ikpy follows the **first** child joint in URDF order: assert the chain ends at `gripper_frame_joint`; clip IK seeds into the joint bounds (else scipy: "x0 is infeasible").
- `import pinocchio` needs `source /opt/ros/jazzy/setup.bash` (apt package).
- A package's first test needs `tests_require=["pytest"]` in `setup.py` and `<test_depend>python3-pytest</test_depend>` in `package.xml`; otherwise `colcon test` falls back to unittest and fails with exit 5 ("NO TESTS RAN").
- A MuJoCo `include` cannot change an included body (raise the base, add a camera): edit a copy with `mujoco.MjSpec`, never `sim/` (ADR-036 update).
- More: `docs/project/13` (runbook) and `04` §8 (kinematics pitfalls).
