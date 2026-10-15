# ADR-024 — Quality tooling: ruff, pre-commit, markdownlint, conventional commits

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | QA → OPS, ARC | C-07, C-50, C-53 |

## Context

Three people write Python, YAML, XML and lots of Markdown. Without shared tooling, style diverges and trivial errors reach review. The image ships `ament_flake8`, `ament_pep257`, `ament_xmllint`, but neither ruff nor pre-commit.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| flake8 + black + isort | Historical tools | Three tools, three configs, slower |
| ament_lint (ament_flake8, ament_pep257, ament_copyright) | ROS standard, integrated in `colcon test` | Per-package config; verbose pep257 rules; overlaps a modern linter |
| **ruff** (lint + format) + **pre-commit** + **markdownlint-cli2** + **conventional-pre-commit** | Fast, autofix, one config (`pyproject.toml`), same config locally and in CI | Tools to install on the host (pip) |

## Decision

ruff (rules E, W, F, I, B, UP, N, SIM, RUF; line length 110; `N803`/`N806` ignored for matrix notation), pre-commit-hooks (YAML/TOML/XML, LF, files > 1 MB, private keys), markdownlint-cli2 (MD013/MD033/MD041 disabled), conventional-pre-commit on `commit-msg`. **Course content (`core/`, `sim/`, `so101_description`, `docs/SUJET.md`, slides), untouched provided files and the spikes are excluded.**

## Consequences

- Team code fixed and formatted (≈ 6 lines); provided files still identical to upstream are excluded instead of modified.
- Versions pinned and updated deliberately (`pre-commit autoupdate` in a PR).

## Evidence / validation

[MES-23]: ruff 0.16.10 → 15 issues and 7 files to reformat before cleanup; 0 on the templates and the provided driver; markdownlint 95 → 0 on the docs (1 real table bug); 13/13 hooks green on the whole repository.

## Reconsider if

ruff covers typing (future rules), or another language appears (C++: add clang-format/clang-tidy).

## Update

After the upstream pull, `docs/SUJET.md` (13 markdownlint issues) and the provided slides were excluded; ruff also excludes `docs/` and `*.md` because it formats Python blocks inside Markdown.

## Update (2026-10-15)

Added after a CI review: **actionlint**, **zizmor** (workflow security), **shellcheck** (warnings) and **hadolint** in pre-commit; **gitleaks** on the whole history in the lint job; a **PR-title** check, since squash merge writes the PR title on `main`. SonarQube Cloud is planned once coverage exists (`14` §4).
