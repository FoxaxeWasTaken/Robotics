# ADR-019 — Test strategy and continuous integration

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED | 2026-10-08 | QA → OPS, all | C-06, C-41, C-42 |

## Context

Several requirements are measurable (rates, accuracy, success rate); the work happens in parallel among three people.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Manual tests | Fast at first | Invisible regressions; nothing proven |
| **Pyramid: pytest (pure) + launch_testing + headless E2E bench + GitHub Actions CI** | Automatic evidence; runs on CPU (OSMesa) | Initial setup |
| 50-episode sim in CI on every commit | Maximal coverage | Too slow: N = 3 per PR, N = 50 nightly |

## Decision

Pyramid of `08` §1, metrics of `08` §2, C-05/C-06/C-23 grep guards, ruff + pre-commit. CI: image build with cache, `colcon build`, `colcon test`, E2E N = 3 per PR and N = 50 nightly (details ADR-025, conventions ADR-026).

## Consequences

- Every PR adds or changes a test.
- Bench results are versioned in `results/`.

## Evidence / validation

Headless feasibility shown by every experiment (OSMesa, CPU); the component test caught a real C-11 violation [MES-22].

## Reconsider if

CI times become annoying: cache the image on GHCR.
