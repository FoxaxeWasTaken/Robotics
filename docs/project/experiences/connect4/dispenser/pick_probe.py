"""pick_probe.py [rail]: /brain/pick's steps on the game's scene with its contacts, the station's token moved
across its slot and the pick aimed off the slot's middle, each by -1.5, 0 and +1.5 mm: the result, and where
the token is across the slot once the jaws closed and at the top of the lift [MES-77].

The slot leaves a token 1.5 mm of play on each side, and the camera places a station within 1.5 mm [MES-76].
The helpers are the dispenser tests': the brain's tick and the driver's at 50 Hz (`drive`).
"""

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[5] / "ros2_ws" / "src" / "so101_bringup" / "test"))
import test_connect4_dispenser as tests  # noqa: E402
from test_connect4_dispenser import (  # noqa: E402
    CAMERA,
    HANDOVER,
    LAYOUT,
    T_DISPENSER,
    URDF,
    Connect4Sim,
    Kinematics,
    Params,
    SO101Sim,
    drop,
    in_dispenser,
    layout,
    run,
    set_arm,
    sim_scene,
    token,
)

rail = int(sys.argv[1]) if len(sys.argv) > 1 else 2
scene = sim_scene.generate(tempfile.mkdtemp(), camera=CAMERA, connect4=LAYOUT)
across, kin, p = T_DISPENSER[:3, 1], Kinematics(str(URDF)), Params()
print(f"station {rail}; PICK_ASIDE {drop.PICK_ASIDE * 1e3} mm, PICK_SEAT {drop.PICK_SEAT * 1e3} mm")
for stands in (-1.5, 0.0, 1.5):
    for aim in (-1.5, 0.0, 1.5):
        sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
        sim.connect()
        set_arm(sim, HANDOVER)
        person = Connect4Sim(sim, LAYOUT, np.random.default_rng(0))
        run(sim, person, 1.0)
        name = token(rail, 0)
        joint = sim.model.joint(f"{name}_free").qposadr[0]
        sim.data.qpos[joint : joint + 3] += across * stands * 1e-3
        run(sim, person, 0.5)
        target = (T_DISPENSER @ [*layout.station(LAYOUT, rail), 1.0])[:3] + across * aim * 1e-3
        move, track = drop.Move(drop.pick_steps(kin, target, np.array(HANDOVER), p), p), {}

        def follow(sim, move=move, track=track, name=name):
            """Called at each tick in place of the tests' contact check: where the token is, by step."""
            at = (in_dispenser(sim, name) - layout.station(LAYOUT, rail)) * 1e3
            track[move.name] = round(float(at[1]), 2)
            return set()

        tests.touching = follow
        result, seconds, _ = tests.drive(sim, person, move)
        print(
            f"token at {stands:+.1f} mm, aimed at {aim:+.1f} mm: {result or 'done'} in {seconds:.1f} s; the token "
            f"across the slot, mm: closed on {track.get('close')}, lifted {track.get('lift_4')}",
            flush=True,
        )
        sim.disconnect()
