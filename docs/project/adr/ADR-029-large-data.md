# ADR-029 — Heavy data outside Git (no LFS); versioned calibration

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | OPS → QA, VIS | C-53 |

## Context

The project produces rosbags (hundreds of MB), videos, test images, CSVs and calibration files. The course slides (10 MB) were duplicated at the root.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Everything in Git | Simple | Huge repo, slow clones, irreversible without history rewrite |
| Git LFS | Versioned large files | 1 GB free quota; complexity for every clone; unnecessary here |
| **Placement rule** (`15` §7) + `check-added-large-files` hook | Light repo; CI artifacts for ephemeral data | Discipline; videos hosted elsewhere |

## Decision

In Git: code, configuration, docs, small reports, **arm calibration JSON** (`so101_bringup/config/calibration/`), test images under 200 KB. Outside Git: rosbags and raw CSVs (14-day CI artifacts or local disk), videos (GitHub Release or link), secrets. Blocking hook above 1 MB; provided content excluded.

## Consequences

- Provided STL meshes stay in Git (course kit).

## Evidence / validation

Inventory of files above 1 MB on 2026-10-08: untracked PDFs, provided meshes.

## Reconsider if

A learning dataset is needed (phase 2, LeRobot policies): Hugging Face Hub, not Git.

## Update

The course now versions the PDF slides in `docs/` (upstream commit `76c55ee`). Rule unchanged for the team: **do not add** other PDFs/media; root copies are duplicates ignored by `/*.pdf`. The provided `docs/` content is excluded from the hooks, so the large-file check never blocks upstream syncs.

## Update (2026-10-30, #189)

One medium joins Git: the README's demo GIF, under 1 MB (the hook's limit), in `docs/project/media/` (#189, `15` §7). Videos stay in a GitHub Release or behind a link, as the robustness demo's clips are.
