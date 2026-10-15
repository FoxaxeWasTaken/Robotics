<!--
One PR-sized ticket of a roadmap step (docs/project/09), written so that a teammate or an AI agent can do it
without asking you: exact files, checkable criteria, what not to touch. Anatomy and Definition of Ready: CONTRIBUTING §8.
Bug? Same sections: Goal = observed vs expected (sim or real arm, commit `git rev-parse --short HEAD`, commands),
Scope = the fix, Done when = the reproduction passes; logs, rosbag or screenshots under References.
Label `task` (or `bug`); lane, priority and size on the GitHub Project; parent epic and "blocked by" under Relationships.
-->

#### Roadmap step

<!-- e.g. 1.3 — camera_info, effort, frames -->

#### Goal

<!-- Why this ticket exists, in one or two sentences (the problem, not the solution). -->

#### Scope

- [ ]

#### Out of scope

<!-- What must not be done here, and which ticket does it. -->

#### Done when

- [ ] <!-- only checkable facts: a command, a test, a measured threshold -->

#### Files and interfaces

<!-- Paths to create or modify; topics, services or parameters touched (interface contract docs/project/01 §3). -->

#### References

<!-- Doc sections, ADRs, requirements (C-xx), measurements (MES-xx). -->

#### Review focus

<!-- What the reviewer checks first (expert role of docs/project/README.md, e.g. [SAFE]). -->

---
_Common rules: AGENTS.md (AI agents), CONTRIBUTING.md §3 (brief rules: only `/joint_command`, no ground truth, no provided extrinsics, kit untouched) and §8 (ticket Definition of Done). Ambiguity in a referenced doc: decide, then update that doc in the same PR._
