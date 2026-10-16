# ADR-026 — Test conventions and anti-flakiness rules

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| DECIDED (complements ADR-019) | 2026-10-08 | QA → CTRL, KIN, VIS, DEC | C-11, C-15, C-41, C-42 |

## Context

ADR-019 sets the test pyramid. Concrete rules are needed: location, colcon declaration, timing tests, sim-specific pitfalls.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| Ad hoc tests per package | Freedom | Each package runs differently; tests ignored by `colcon test` |
| **Shared conventions** (`test/`, `tests_require`, `test_depend`, unit vs component, flaky quarantine) | One `colcon test` for everything; simple CI | Discipline |

## Decision

`<pkg>/test/test_*.py` (unit pytest) and `test_*_launch.py` (launch_testing). `setup.py`: `tests_require=["pytest"]`; `package.xml`: `python3-pytest`, `launch_testing_ros`. Timing tests measured over ≥ 3 s after a 1 s warm-up. **Clamping and any rule MuJoCo already applies are unit-tested**, not in sim. No flaky test in `main`: quarantine + issue within a week.

## Consequences

- `ros2_ws/src/so101_driver/test/` holds the reference examples.
- Small test data, regenerable by script.

## Evidence / validation

[MES-22]: `colcon test` runs 6 tests in 9.4 s; the rate test revealed **9 Hz** (4.7 Hz under test) on the early driver; 30 Hz after the fix [MES-24].

## Reconsider if

Component tests make CI too slow: mark them `slow` and run them nightly only.
