# ADR-031 — Repository license

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| OPEN | 2026-10-08 | CP → teaching staff | C-53 |

## Context

The repository mixes code derived from LeRobot (Apache-2.0), packages declared Apache-2.0 or MIT, and a course kit without a stated license. The repository should eventually become public for CVs.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| No license | — | All rights reserved by default: nobody may legally reuse the code |
| **Apache-2.0** for the team's code | Consistent with LeRobot and the kit; patent clause | Must coexist with the kit's terms |
| MIT | Very permissive | Less consistent with the rest |

## Decision

**Proposal**: Apache-2.0 for the team's code, keeping LeRobot's headers and attribution. **To confirm with the teaching staff before any publication.**

## Consequences

- Until settled, the repository stays private.

## Evidence / validation

License inventory (`15` §9).

## Reconsider if

The teaching staff answers.
