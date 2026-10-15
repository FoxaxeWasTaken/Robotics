# Changelog

All notable changes. Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions: one tag per functional milestone (`docs/project/15` §6).

## [Unreleased]

### Added

- Repository architecture: CI (lint, guards, build, tests), nightly, school mirror, governance files, project documentation.
- Planning conventions: GitHub Project (epics, PR-sized tickets, fields, automations), ticket-anatomy issue form, `Closes #` in the PR template; `AGENTS.md` (+ `CLAUDE.md` import) for AI coding agents.
- Phase 2: Connect Four against the robot, validated by the teaching staff (`docs/project/phase2/`, ADR-034 to 038), with its own roadmap (`phase2/P2-04`) and its kinematics and camera measurements (MES-27 to 32); store-bought grid on a riser, depth guide, token handover, wrist camera (ADR-038); real-arm calibration-zero check (MES-30, `07` §4).

### Changed

- Code: the repository starts from the course stubs (upstream `42737bf`), with a 3-line lint fix; the bootstrap prototype stays outside the repository (ADR-039).
- CI: the control job passes when no package has tests yet.
