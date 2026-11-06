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

## Update (2026-11-02, #210): a minor version per usable increment, one milestone per version

One tag in a month (`v1.0.0`) was too few: a version was a large functional milestone, with no ticket and no workflow step to bring it up, and the `v0.x` milestones went by unnoticed. The rule from now on:

- **A minor version per usable increment**: what a member or the defense can use once it is merged and the nightly passed it. Planned: `v1.1.0` real-arm tooling, `v1.2.0` real camera and perception, `v1.3.0` real pick & place (C-43), `v2.x` phase 2. A **patch** for a fix that matters on its own.
- **Each version is a GitHub milestone**. Every task ticket belongs to one from its creation (CONTRIBUTING §8), and each open milestone has its "Release vX.Y.Z" ticket.
- **A release** (`15` §6) closes its milestone once its other tickets are closed, and opens the next minor's if it is missing.
- **Enforced by #211**: a PR cannot merge when a ticket it closes has no milestone; `tools/release.sh` refuses a milestone with open tickets; the nightly report says when `main` is releasable.

The milestones were put in order on 2026-11-02: `v0.1.0`–`v0.3.0` (never tagged) and `v1.0.0` (released) closed, with each closed ticket filed under the version it shipped in.

## Update (2026-11-06)

`v1.2.0` is not tagged: the real camera and the real pick & place came in one session, so `v1.3.0` is the final phase-1 release and its milestone took the tickets of `v1.2.0` (#213, #214).
