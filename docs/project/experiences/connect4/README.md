# Connect Four prototypes (2026-11-07)

What the expert review of phase 2 ran to measure the sim, kept as it ran. **Throwaway, not production code**: each ticket ports what it needs, cleanly, with tests. Each folder has its own scratch scene and its own layout: #76 puts them on one, the layout of `phase2/P2-02` §7.

| Folder | Question | Measurement | Starts ticket |
| --- | --- | --- | --- |
| `scene/` | Riser, contacts, tick with 42 tokens, drops, grip, repark | MES-63 | #76, #79, #80 |
| `drop/` | Pose of a drop, paths, clearance, grip, duration, async service; `e13_grid_clearance.py` and `e14_edge_on.py` (2026-11-13), on the package's scene: how near the grid a move passes and what the brain's model says of it, the grid set down elsewhere; how far from edge-on a drop and a pick still work | MES-64, MES-78, MES-79 | #83, #84, #85, #260 |
| `reading/` | Camera, pixels per cell, classification, grid pose accuracy | MES-65 | #81, #82 |
| `game/` | Engine, turns, console client, fake stack; `levels.py`: the package's engine, depth against depth; `play_keys.py` and `record_debug.py`: a game through the console on a pipe, the README's game GIF, the phase-2 video and the image of a circled four from `/connect4/debug_image` | MES-66, MES-70 | #78, #230, #86, #232, #244, #246 |
| `dispenser/` (2026-11-13) | Where the arm can take a token, what the open jaws need free, a gravity rail's feed and a pick with real contacts; `wake_probe.py`: what stalls a sleeping queue; `render_overview.py`: the README's picture of the scene; `pick_probe.py`: the pick with the token and the aim off the slot's middle; `pose_noise.py`: the measured poses image by image during a game | MES-75, MES-77 | #257, #259 |

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

# #81: the reference images of so101_connect4/test/data, rendered from the game's scene (the package cannot
# import the sim); run it again when the scene or connect4_grid.yaml changes
tools/so101 run --out ros2_ws/src/so101_connect4/test/data \
  'python3 /repo/docs/project/experiences/connect4/reading/render_reference.py'

# game/: pure Python, on any machine
cd docs/project/experiences/connect4/game && python3 test_game.py && python3 test_turns.py && python3 bench_engine.py depth
# #244: the levels, the package's engine against itself (~4 min)
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/game/levels.py 40 1,4 4,10 1,10 2,6 6,10 2,10'
# #246: one game through the console on a pipe, its debug images recorded; then the GIF, on the host (Pillow)
tools/so101 run --game --out /tmp/frames 'cd /repo/docs/project/experiences/connect4/game &&
  (python3 record_debug.py record /out > /dev/null &) ; python3 play_keys.py "e 1,m 7,h ? 1,7,1,7,1"'
python3 docs/project/experiences/connect4/game/record_debug.py gif /tmp/frames docs/project/media/demo_connect4.gif
# the README's GIF (#259): the keys "h 1,7,1,7,1,7", then one image in 4 (the game at 16x, 0.99 MB)
python3 docs/project/experiences/connect4/game/record_debug.py gif /tmp/frames docs/project/media/demo_connect4.gif 4
# #97: the phase-2 video, a bench game with every debug image kept, then the MP4 at 4x from image 50 (27 s, 8.8 MB;
# a Release asset, never in Git)
tools/so101 run --out /tmp/video '(ros2 launch so101_bringup sim.launch.py expose_ground_truth:=true \
  perception:=true connect4:=true > /out/stack.log 2>&1 &) ;
  (python3 /repo/docs/project/experiences/connect4/game/record_debug.py record /out/frames 1 > /dev/null &) ;
  ros2 run so101_bringup evaluate --games 1 --out /out/game.csv; sleep 4'
tools/so101 run --out /tmp/video 'python3 /repo/docs/project/experiences/connect4/game/record_debug.py \
  mp4 /out/frames /out/demo_connect4_sim.mp4 50'
# #257: where a token can be taken; then one rail, 7 tokens fed and taken (azimuth, r, z, slope, tokens, the
# tokens' angular damping or -1 for the scene's, sleep, jaws' opening %, the wake's force in N)
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/dispenser/reach.py'
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/dispenser/rail.py 60 0.27 0.02 25 7 -1 1 40 1e-6'
# the README's picture of the scene, from a free camera (960x720)
tools/so101 run --out docs/project/media 'python3 /repo/docs/project/experiences/connect4/dispenser/render_overview.py'
# what stalls a sleeping queue, one variable at a time (the package's scene, with its dispenser)
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/dispenser/wake_probe.py'
# #259: the pick at a station (0 to 2), the token and the aim each -1.5, 0 and +1.5 mm off the slot's middle
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/dispenser/pick_probe.py 2'
# the measured poses against the layout's, for 150 s, in a container where the game's stack runs and a game is played
python3 docs/project/experiences/connect4/dispenser/pose_noise.py 150
# #260: how near the grid a game's moves pass, in truth and as the brain judges it; the grid moved, then 30 random places
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/drop/e13_grid_clearance.py moved wide 30'
# a drop on a grid turned about its column, a pick at a dispenser turned about its station, -8 to +8 degrees
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/drop/e14_edge_on.py grid'
tools/so101 run 'python3 /repo/docs/project/experiences/connect4/drop/e14_edge_on.py dispenser'
```

`reach.npy` (3.7 MB) is generated, never committed. `drop/e5_async.py` is the test of a service that replies at the end of a move while the 50 Hz timer runs.
