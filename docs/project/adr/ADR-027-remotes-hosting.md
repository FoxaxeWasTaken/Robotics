# ADR-027 — Hosting: team repository + course `upstream` remote

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | CP → OPS | C-07, C-53 |

## Context

The current clone's `origin` is **the teacher's repository**. The team has no repository yet.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| GitHub fork | Visible link with the original | A fork of a public repo stays public; its PRs target the teacher by default |
| **New team repository + read-only `upstream`** | Visibility of choice; history kept; sync by merge | Manual remote setup |
| Copy without history | — | History lost, hard sync |

## Decision

`git remote rename origin upstream`; `git remote set-url --push upstream DISABLED` (safety net); new **private** team repository during the project, public at the end after the staff's approval. Sync: `git fetch upstream && git merge upstream/main` in a `chore: sync upstream` PR. The prepared tree is pushed by copying it into the existing clone (history kept, `15` §11).

## Consequences

- README and CONTRIBUTING document the procedure.
- Never modify the provided kit, to keep merges easy (proven by the PR #1 pull).

## Evidence / validation

`git remote -v` on 2026-10-08.

## Reconsider if

The staff imposes a submission mode (GitHub Classroom, provided repository).
