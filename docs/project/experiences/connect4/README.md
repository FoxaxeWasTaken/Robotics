# Connect Four prototypes (2026-11-07)

What the expert review of phase 2 ran to measure the sim, kept as it ran. **Throwaway, not production code**: each ticket ports what it needs, cleanly, with tests. Each folder has its own scratch scene and its own layout: #76 puts them on one, the layout of `phase2/P2-02` §7.

| Folder | Question | Measurement | Starts ticket |
| --- | --- | --- | --- |
| `scene/` | Riser, contacts, tick with 42 tokens, drops, grip, repark | MES-63 | #76, #79, #80 |
| `drop/` | Pose of a drop, paths, clearance, grip, duration, async service | MES-64 | #83, #84, #85 |
| `reading/` | Camera, pixels per cell, classification, grid pose accuracy | MES-65 | #81, #82 |
| `game/` | Engine, turns, console client, fake stack | MES-66 | #78, #230, #86, #232 |

## Running

From the repository root, with the `tek5/control` image built:

```bash
# scene/: run.sh mounts its own folder and writes there; reach.py first, `move` seeds from its map
docs/project/experiences/connect4/scene/run.sh drop
tools/so101 run --out docs/project/experiences/connect4/scene 'python3 -u /out/drv.py'
# #76, one layout (P2-02 §7): the go / no-go rates, the capture, the clearance; then full grids read back (~2 min)
tools/so101 run 'python3 -u /repo/docs/project/experiences/connect4/scene/exp_connect4_scene.py spike'
tools/so101 run 'python3 -u /repo/docs/project/experiences/connect4/scene/exp_connect4_scene.py stacks'

# drop/ and reading/: the sources in /repo, the folder in /out
tools/so101 run --out docs/project/experiences/connect4/drop 'cd /out && python3 -u e7_cycle.py'
tools/so101 run --out docs/project/experiences/connect4/reading 'cd /out && python3 -u final.py'

# game/: pure Python, on any machine
cd docs/project/experiences/connect4/game && python3 test_game.py && python3 test_turns.py && python3 bench_engine.py depth
```

`reach.npy` (3.7 MB) is generated, never committed. `drop/e5_async.py` is the test of a service that replies at the end of a move while the 50 Hz timer runs.
