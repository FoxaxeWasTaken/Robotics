# Changelog

All notable changes. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions: one tag per functional milestone (`docs/project/15` §6).

## [Unreleased]

### Added

- Base driver (step 1.0): `/joint_states` at 30 Hz (gripper in rad, stamped at read time), `/joint_command` with partial and unnamed commands, a speed limiter (1.5 rad/s, gripper 200 %/s) in a pure `limiter.py`; unit and `launch_testing` tests. The real arm stays limp until 6.1a.
- `so101_common`: joint names, joint limits, gripper %↔rad conversion and frame names defined once, tested against the URDF (ADR-003); the driver and brain stubs drop their local copies.
- Repository architecture: CI (lint, guards, build, tests), nightly, school mirror, governance files, project documentation.
- Planning conventions: GitHub Project (epics, PR-sized tickets, fields, automations), a single ticket-anatomy issue template, `Closes #` in the PR template; `AGENTS.md` (+ `CLAUDE.md` import) for AI coding agents.
- Phase 2: Connect Four against the robot, validated by the teaching staff (`docs/project/phase2/`, ADR-034 to 038), with its own roadmap (`phase2/P2-04`) and its kinematics and camera measurements (MES-27 to 32); store-bought grid on a riser, depth guide, token handover, wrist camera (ADR-038); real-arm calibration-zero check (MES-30, `07` §4).

### Changed

- CI: nothing runs on the school copy, Dependabot PRs included: the Dependabot exception of `ci.yml` and `pr-title.yml` is limited to this repository.
- Code: the repository starts from the course stubs (upstream `42737bf`), with a 3-line lint fix; the bootstrap prototype stays outside the repository (ADR-039).
- CI: the control job passes when no package has tests yet; PR title checked as a conventional commit; actionlint, zizmor, shellcheck and hadolint in pre-commit, gitleaks on the whole history, `ament_xmllint`; actions pinned by SHA, runs on `main` never cancelled, checks also run on Dependabot PRs.
- Mirror: force-pushes `main` and the tags (exact copy); key in the `main`-only `school-mirror` environment.
- Contributing: naming rules for branches, commits and PR titles (CONTRIBUTING §2.1); AGENTS.md: safety, push, evidence and isolation rules.
