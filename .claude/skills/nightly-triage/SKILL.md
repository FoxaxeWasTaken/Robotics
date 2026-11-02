---
name: nightly-triage
description: Triage this project's latest nightly results, the bench episodes, the quality trends and the SonarQube issues, and open one ticket per real regression. Use when asked to check the nightly, after a red or degraded night, or before a release.
---

# Triage the nightly

The jobs and their reports are described in `docs/project/14` §6, the bench target in `docs/project/08` §2, and the SonarQube policy in `14` §4. Always pass `-R FoxaxeWasTaken/Robotics` to `gh`.

1. **Read the reports**: the last comments of the pinned issue "Nightly bench results" (label `report`; `gh issue list -R FoxaxeWasTaken/Robotics --label report`). There is one bench comment and one quality comment per night. Compare with the nights before.
2. **Failed episodes**: find their cause in the run's artifacts (the CSV, the rosbag of each failed episode). Sort them: the stack (a real bug), the runner (CPU, a timeout with no wrong state), or the scene (an unreachable cube). Replay a suspect seed in a throwaway container (`isolated-stack` skill).
3. **Trends**: compare `trends.csv` (IK benchmarks, the driver's timer period) with the previous nights. Do not chase one noisy point: runners are noisy, so there is no threshold (`14` §6). Act on a step that lasts.
4. **SonarQube issues**: fix a real one in a PR. Mark a false positive, or an accepted issue, with its reason in SonarQube, and add that reason to the list of `14` §4.
5. **Tickets**: one per real regression, with the `new-ticket` skill. Use the `bug` label, and put the run link and the reproduction under References. A flaky test follows CONTRIBUTING §5.
