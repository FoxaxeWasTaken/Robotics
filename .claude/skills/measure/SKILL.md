---
name: measure
description: Record a new measurement (MES) or decision (ADR) in this project's design docs. Use when a spike, a benchmark or an experiment produces a number that a decision or a doc relies on, or when a design decision is taken or changed.
---

# Record a measurement or a decision

## A measurement (MES)

The register and its format are in `docs/project/11` §4; `AGENTS.md` (Conventions) says where scripts go.

1. **The script** goes in `docs/project/experiences/`, never in a package. Its docstring holds what it measures and the exact command that replays it (container, arguments). Sim scripts run in a throwaway container (`isolated-stack` skill).
2. **Run it**, and keep the output for the PR.
3. **The row**: the next free `MES-xx` in `11` §4, with what was measured, the numbers and the script.
4. **The indexes**: the row in `docs/project/experiences/README.md`, and the counts in `docs/project/README.md`.
5. **The citations**: `[MES-xx]` in the doc whose claim it supports, and in the PR.

## A decision (ADR)

The index is `docs/project/adr/README.md`.

- **New decision**: the next free `ADR-0xx-<topic>.md`, its line in the index, the counts in `docs/project/README.md`.
- **Change to an accepted ADR**: add an `## Update (<date>, #<ticket>)` section, or write a new ADR that supersedes it. Never rewrite its decision (`AGENTS.md`).
- Then update the docs that state the old decision (CONTRIBUTING §6).
