# ADR-020 — Git workflow

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (visible history) + DECIDED | 2026-10-08 | CP → QA | C-07, C-50 |

## Context

The code must be on Git with each member's history visible; everyone must understand the whole pipeline.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| One integrator pushing for everyone | Clean history | **Violates C-07** |
| **Short branches + reviewed PRs + conventional commits** | Per-author history; review = knowledge sharing | Discipline required |
| Direct pushes to `main` | Fast | Conflicts, regressions |

## Decision

Protected `main` (PR, 1 approval, green CI). Everyone commits their own work; `Co-authored-by` when pairing. Conventional commits, **in English**. Tags per functional milestone. **Rotating** reviews across lanes.

## Consequences

- The work in progress (driver, brain, RViz) is merged with upstream and prepared in the repository tree; commit it through PRs.

## Evidence / validation

`08` §6, `15` §6.

## Reconsider if

—
