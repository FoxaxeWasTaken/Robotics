# ADR-015 — Explicit hand-written state machine

| Status | Date | Experts (author → reviewers) | Related requirements |
| --- | --- | --- | --- |
| IMPOSED (explicit FSM) + DECIDED | 2026-10-08 | DEC → QA, KIN | C-32, C-34, C-35 |

## Context

The brief imposes an explicit state machine, the slides the `/brain_state` publication.

## Options considered

| Option | Pros | Cons |
| --- | --- | --- |
| **Enum + transition table, pure `fsm.py` module** | Explicit, ROS-free testable, zero dependency | Tooling to write (logs, visualization) |
| SMACH / FlexBE | Introspection | Poorly maintained on Jazzy, dependencies |
| Behavior trees (py_trees, BT.CPP) | State of the art, reactive | Not an explicit FSM; overkill |

## Decision

Hand-written FSM: states and transitions of `06` §3.1, 50 Hz tick, injected clock, `/brain_state` transient_local, `/brain/start`, `/brain/stop`, `VERIFY`, `RETRY`, `ERROR`, `STOPPED` states.

## Consequences

- One unit test per transition.
- Long operations go through futures (ADR-017).

## Evidence / validation

Design review; `VERIFY` needed per [MES-09].

## Reconsider if

Phase 2 brings composite missions (sorting, multiple objects): behavior tree.
