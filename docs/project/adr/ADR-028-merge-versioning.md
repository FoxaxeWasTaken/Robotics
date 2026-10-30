# ADR-028 — PR merges, versions and changelog

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | CP → QA | C-07, C-53 |

## Context

We need a readable history attributable to each member, and versions matching the functional milestones.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Merge commit | Full branch history | Spaghetti history, noisy intermediate commits |
| Rebase + fast-forward | Linear, keeps every commit | Rewrites shared branches; more rigor needed |
| **Squash merge** | 1 PR = 1 clean commit; author = PR author; `Co-authored-by` kept | Intermediate commits lost on `main` (kept in the PR) |

## Decision

**Squash merge** of PRs; PR title = conventional message. **SemVer tags per functional milestone** (`v0.1.0` driver … `v1.0.0` validated sim pick & place, `v1.1.0` real arm, `v2.x` phase 2). `CHANGELOG.md` in Keep a Changelog format, updated in the PR preparing the tag. One GitHub Release per tag (notes, bench report, video link).

## Consequences

- Option: changelog generated from commits (git-cliff) if manual upkeep gets heavy.

## Evidence / validation

Repository hygiene review (`15` §6).

## Reconsider if

The team prefers keeping every work commit (rebase + fast-forward).

## Update (2026-10-30, the first tag)

No `v0.x` tag was made: the driver, IK and perception milestones went by without one. The first tag is `v1.0.0`, the simulation milestone (#63); the next ones follow the decision (`v1.1.0` real arm, `v2.x` phase 2).
