# Changelog

All notable changes. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions: one tag per functional milestone (`docs/project/15` §6).

## [Unreleased]

### Added

- Driver, rest of stage 1: `/joint_command` clamped to the joint limits and rejected whole on a size mismatch or a NaN/inf, in a pure `command.py` with property tests (1.1a); rates, speed limits and image size as read-only ROS parameters with ranges, values in `so101_bringup/config/driver.yaml` (1.1b); `/external_cam/camera_info` with the image's header, `frame_id` = `external_cam_optical_frame` (1.3a); normalized effort in `/joint_states`, gripper positive when squeezing (1.3b, MES-35).
- `perception` service in `docker-compose.yml` (C-02), on the same DDS domain and transport as `control`; idle until step 3.1. `docker compose up -d control perception viz` now matches the brief, and the E2E and nightly jobs can start the stack.
- `so101_bringup` (step 0.3): `sim.launch.py` starts the driver and the brain (`use_sim` and `port` forwarded), parameter files in `config/`, and the RViz file moved to `rviz/so101.rviz`; the control container starts it. A `launch_testing` test checks both nodes and the forwarded arguments.
- Sim camera (step 1.2): `/external_cam/image_raw` (`rgb8`, 320×240, 10 Hz) rendered in a dedicated thread on a copy of the sim state, behind a backend lock; OSMesa render threads limited to half the cores so the control period keeps p99 20.4 ms (MES-34). `launch_testing` checks the image and a clean SIGINT stop.
- Test tooling (0.3d): Hypothesis property tests, coverage with a comment on each PR (python-coverage-comment-action, non-blocking), and colcon-ros-domain-id-coordinator so `colcon test` never shares a DDS domain with a running stack or a parallel run; one 11.5 MB layer in both images.
- Base driver (step 1.0): `/joint_states` at 30 Hz (gripper in rad, stamped at read time), `/joint_command` with partial and unnamed commands, a speed limiter (1.5 rad/s, gripper 200 %/s) in a pure `limiter.py`; unit and `launch_testing` tests. The real arm stays limp until 6.1a.
- `so101_common`: joint names, joint limits, gripper %↔rad conversion and frame names defined once, tested against the URDF (ADR-003); the driver and brain stubs drop their local copies.
- Repository architecture: CI (lint, guards, build, tests), nightly, school mirror, governance files, project documentation.
- Planning conventions: GitHub Project (epics, PR-sized tickets, fields, automations), a single ticket-anatomy issue template, `Closes #` in the PR template; `AGENTS.md` (+ `CLAUDE.md` import) for AI coding agents.
- Phase 2: Connect Four against the robot, validated by the teaching staff (`docs/project/phase2/`, ADR-034 to 038), with its own roadmap (`phase2/P2-04`) and its kinematics and camera measurements (MES-27 to 32); store-bought grid on a riser, depth guide, token handover, wrist camera (ADR-038); real-arm calibration-zero check (MES-30, `07` §4).

### Changed

- Contributing: a PR may group related tickets (same files or same roadmap step), with one commit and one `Closes #` line per ticket; WIP limit raised from 2 to 5 tickets In progress per person (AGENTS.md, CONTRIBUTING, PR template, `15` §12).
- CI: nothing runs on the school copy, Dependabot PRs included: the Dependabot exception of `ci.yml` and `pr-title.yml` is limited to this repository.
- Docs: the teaching-staff answers of 2026-10-16 are recorded (project README, `phase2/P2-03` §4) and applied to ADR-002, 013, 018, 031, 033, 036 and 038, `00`, `05` §6, `07` §3.1, `09`, `10` and `P2-02`; roadmap steps 5.3 (release v1.0.0) and 5.4 (robustness demo of the five [BOOT2 p12] cases, asked by the staff).
- Code: the repository starts from the course stubs (upstream `42737bf`), with a 3-line lint fix; the bootstrap prototype stays outside the repository (ADR-039).
- CI: the control job passes when no package has tests yet; PR title checked as a conventional commit; actionlint, zizmor, shellcheck and hadolint in pre-commit, gitleaks on the whole history, `ament_xmllint`; actions pinned by SHA, runs on `main` never cancelled, checks also run on Dependabot PRs.
- Mirror: force-pushes `main` and the tags (exact copy); key in the `main`-only `school-mirror` environment.
- Contributing: naming rules for branches, commits and PR titles (CONTRIBUTING §2.1); AGENTS.md: safety, push, evidence and isolation rules.

### Fixed

- Control container shutdown: SIGINT goes to `ros2 launch` only, which forwards one to each node; signalling the whole process group gave each node a second SIGINT that interrupted `destroy_node()` (traceback on every `docker compose stop`).
- Driver: SIGINT or SIGTERM while a timer callback publishes no longer ends in a traceback and exit code 1 (seen once in 40 SIGTERM stops).
