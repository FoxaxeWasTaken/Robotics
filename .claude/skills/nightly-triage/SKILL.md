---
name: nightly-triage
description: Triage this project's latest nightly results, the bench episodes, the quality trends and the SonarQube issues, and open one ticket per real regression. Use when asked to check the nightly, after a red or degraded night, or before a release.
---

# Triage the nightly

The jobs and their reports are described in `docs/project/14` §6, the bench targets in `docs/project/08` §2 (pick & place) and `docs/project/phase2/P2-01` §4 (the games, P2-06), and the SonarQube policy in `14` §4. Always pass `-R FoxaxeWasTaken/Robotics` to `gh`.

1. **Read the reports**: the last comments of the pinned issue "Nightly bench results" (label `report`; `gh issue list -R FoxaxeWasTaken/Robotics --label report`). There is one bench comment and one quality comment per night. Compare with the nights before.
2. **Failed episodes**: find their cause in the run's artifacts (the CSV, the rosbag of each failed episode). Sort them: the stack (a real bug), the runner (CPU, a timeout with no wrong state), or the scene (an unreachable cube). Replay a suspect seed in a throwaway container (`isolated-stack` skill). **Failed games** the same way: the `games` line of the report, `eval_games.csv` (its `cause`: `fix_board:<reason>`, `token_lost`, `wrong_column`, `arm_moved`, `bench_timeout`; `first` and `history` replay the game) and the rosbags in `bags_games/` (`08` §3.1).
3. **Trends**: compare `trends.csv` (IK benchmarks, the game engine's move at the hard level, the driver's timer period) with the previous nights. Do not chase one noisy point: runners are noisy, so there is no threshold (`14` §6). Act on a step that lasts.
4. **SonarQube issues**: fix a real one in a PR. Mark a false positive, or an accepted issue, with its reason in SonarQube, and add that reason to the list of `14` §4.
5. **The release line**: the bench report's last line says which version is next and whether `main` is releasable. "Releasable": propose the release (`release` skill). "Not yet": the open tickets it names are the version's remaining work.
6. **Tickets**: one per real regression, with the `new-ticket` skill. Use the `bug` label, and put the run link and the reproduction under References. A flaky test follows CONTRIBUTING §5.
