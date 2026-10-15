# ADR-030 — Versioned documentation: Markdown, checked Mermaid, ADRs, English

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | ARC → QA, CP | C-50, C-53 |

## Context

Shared understanding is imposed (C-50) and the repository must be documented (C-53). This folder contains many diagrams and decisions; commits and presentations are in English.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| GitHub wiki / Google Docs | Easy editing | Outside the repository, not reviewed in PRs, drifts from the code |
| MkDocs Material site | Navigation, search | Infrastructure to maintain; unnecessary while GitHub renders Markdown |
| **Markdown in the repository + Mermaid + ADRs**, checked in CI | Reviewed in PRs like code; versioned diagrams; native GitHub rendering | No full-text search (outside GitHub) |

## Decision

Documentation lives in `docs/project/` (design, roadmap, measurements), ADRs in `docs/project/adr/`, **written in English** (consistent with commits and the defense). Diagrams in **Mermaid**, **rendered in CI** by mermaid-cli (`tools/ci/check_mermaid.sh`), markdownlint on every commit, external links checked nightly (lychee). A PR changing documented behavior updates the docs in the same PR.

## Consequences

- Later option: publish `docs/project/` as an MkDocs site for the defense.
- Mermaid pitfalls found: `;` is a statement separator, `=` is escaped in gitGraph ids, avoid `<`/`>` in labels.

## Evidence / validation

mermaid-cli 12.0.1: every diagram rendered; markdownlint: 0 issue [MES-23].

## Reconsider if

The documentation volume justifies full-text search (MkDocs).
