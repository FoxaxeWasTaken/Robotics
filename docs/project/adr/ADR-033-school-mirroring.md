# ADR-033 — Private repository with CI, mirrored to the school repository

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | OPS → CP, QA | C-07, C-53 |

## Context

CI, PRs and protections must live on a private repository controlled by the team; the submission happens on the school repository, which must reflect the history (C-07) without being modified there. Complements ADR-027.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| **`mirror.yml` workflow** (push `main` + tags, deploy key) | Automatic, only pushes the reviewed state, workstation-independent | Key and variables to configure once |
| Local double push URL | No GitHub configuration | Depends on every workstation, pushes work branches |
| `git push --mirror` | Exact copy | Pushes/deletes every ref; fails on PR refs |

## Decision

`mirror.yml` pushes **`main` and `v*` tags**, **never forced**, through a **deploy key** with write access limited to the school repository. Workflows are opt-in (`CI_ENABLED`, `SCHOOL_REPO_SSH_URL`) so nothing runs on the mirror.

## Consequences

- Any direct write to the school repository breaks the mirror: intended, the failure must be visible.
- Nothing secret in `main`.

## Evidence / validation

actionlint 1.7.12: 0 error. To validate for real with a `workflow_dispatch` (`16` §5).

## Reconsider if

The school offers pull mirroring, or imposes another submission mode (GitHub Classroom).

## Update (2026-10-15)

- **Forced push**: nobody pushes to the school repository, so `mirror.yml` now force-pushes `main` and the tags; the school side is an exact copy of the reviewed private `main` (team decision). The private `main` itself still forbids force-pushes (ruleset).
- **Key**: the team has push but not admin rights on the school repository, so no deploy key there: the public key is on the member's account and the secret lives in the `school-mirror` environment, restricted to `main` (`16` §5). For the same reason the workflow no longer runs on tag pushes: tags go with the next `main` push or a manual run.

## Update (2026-10-16)

The deploy-key question to the teaching staff is withdrawn (no admin rights there, see the update above). `dependabot.yml` is mirrored, so Dependabot may open PRs on the school copy: they are closed by hand.

## Update (2026-11-02, #216): nothing lives on the school repository

The mirror also deletes every branch of the school repository but `main`, at each run and daily (`tools/ci/prune_school.sh`). Deleting a PR's head branch makes GitHub close that PR. A Dependabot PR there, or a branch anyone pushed, no longer waits for someone to close it by hand. A refused Dependabot PR on the team side would otherwise have stayed open on the school side for good. The key's rights are unchanged (push, no API): a PR from a fork and the issues stay out of reach, to close by hand if they ever happen. `dependabot.yml` is still mirrored: `main` stays the same commit on both sides (`16`).
