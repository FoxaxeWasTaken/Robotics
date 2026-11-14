# ADR-031 — Repository license

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (Update of 2026-11-14) | 2026-10-08 | CP → teaching staff | C-53 |

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

## Update (2026-10-16)

Not asked for now (team decision): the repository is private, so no license is needed yet. Stays OPEN; to settle before any public release (8.1c).

## Update (2026-11-14)

Settled (team decision), one week before the hand-in and the public switch (#99):

- **Apache-2.0 for the team's work**: the `LICENSE` file at the root, the standard text. It is what every manifest of our packages already declares.
- **A `NOTICE` file says what the license does not cover**: `core/lerobot_min` (LeRobot, Apache-2.0, headers kept) and the course kit (`sim/`, `so101_description`, `example_py_pkg`, `docs/SUJET.md`, the course slides), which states no license.
- **The kit's terms**: the school agreed that the kit, its brief included, is published with the repository (answer to the team, November 2026). The kit stays unmodified and under its own terms.

The repository stays private until #99 makes it public.
