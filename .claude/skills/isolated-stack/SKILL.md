---
name: isolated-stack
description: Run this project's tests, the simulation, the bench or a one-off script next to a developer's running stack without touching it, and render demo media (videos, slide images) from a run. Use before running anything ROS or Docker on a machine where someone else's stack may be up.
---

# Run next to a running stack

The rule is in `AGENTS.md`: never stop or restart someone's containers, and never use their ROS domain (`docker/.env`, 69 by default). The domains are listed in the README's "ROS domains" table.

## Pick the command

| Need | Command | Isolation |
| --- | --- | --- |
| Tests of one package or all of ours | `tools/so101 test [package [pytest options]]` | a new control container; `colcon test` gives each package a free domain. It builds in the shared `ros2_ws/build/`: when a developer's stack is up, prefer the next row |
| Tests, a scenario, a capture, a recording | `tools/so101 run [--stack] [--out DIR] '<command>'` | a throwaway container: no network, domain 79, the repository read-only, our packages copied and built there; `--stack` starts `sim.launch.py` first and waits for the brain; `DIR` is mounted at `/out` |
| The evaluation bench | `tools/so101 bench --stub\|--perception [-n N]` | domain 77; it refuses to start when a stack already answers there |

`--dry-run` prints the `docker` line without running it.

Example, a test with pytest options:

```bash
tools/so101 run 'colcon test --packages-select so101_brain --event-handlers console_direct+ --pytest-args -k fsm && colcon test-result --verbose'
```

## Demo media

- **Video**: record in the container (`--out DIR`). Encode with `cv2.VideoWriter`: `mp4v` (MP4) and `VP90` (WebM) work in the image, `avc1` does not.
- **Slide images**: render them from the run's images, not from a screen capture.
- **What goes in Git**: files over 1 MB stay out (`docs/project/15` §7). Videos go to a GitHub Release, and the docs link to them.

## Known trap

A `ros2 launch … &` started from a non-interactive shell ignores SIGINT. Stop it with SIGTERM, or let the throwaway container end. `tools/so101 run` does the latter.
