"""Reference images of the game's scene for so101_connect4/test/test_board.py (#81): the phase-1 camera at
640x480, the arm at HANDOVER, tokens settled in their cells; reference.json holds each image's true cells and K.
so101_connect4 cannot import the sim (C-05), so the images are rendered here and committed.

    tools/so101 run --out ros2_ws/src/so101_connect4/test/data \
        'python3 /repo/docs/project/experiences/connect4/reading/render_reference.py'
"""

import json
import tempfile
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_bringup import sim_scene
from so101_connect4 import layout
from so101_driver.camera import opencv_intrinsics
from so101_driver.connect4_sim import Connect4Sim

OUT = Path("/out")
HANDOVER = [-1.0, -0.6, 0.6, 1.1, 1.57]
LAYOUT = layout.load()
T_GRID = layout.grid_pose(LAYOUT)
DISC_IN_SLOT = np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])


def boards(rng):
    """name: cells (row-major from the bottom). Empty, a first move, two games under way, a full grid."""
    out = {"empty": [0] * 42, "first_move": [0, 0, 0, 1, 0, 0, 0] + [0] * 35}
    for name, moves in (("midgame", 17), ("late", 34), ("full", 42)):
        cells, heights, player = [0] * 42, [0] * 7, 1
        for _ in range(moves):
            column = int(rng.choice([c for c in range(7) if heights[c] < 6]))
            cells[heights[column] * 7 + column] = player
            heights[column] += 1
            player = 3 - player
        out[name] = cells
    return out


def render(name, cells, camera):
    sim = SO101Sim(scene_path=sim_scene.generate(tempfile.mkdtemp(), camera=camera, connect4=LAYOUT))
    sim.camera_width, sim.camera_height = 640, 480
    sim.connect()
    for joint, value in zip(ARM_JOINTS, HANDOVER, strict=False):
        sim.data.qpos[sim.model.joint(joint).qposadr[0]] = value
        sim.data.ctrl[sim.model.actuator(joint).id] = value
    used = {1: 0, 2: 0}
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, (T_GRID[:3, :3] @ DISC_IN_SLOT).ravel())
    for i, cell in enumerate(cells):
        if cell:
            token = f"token_{'red' if cell == 1 else 'yellow'}_{used[cell]}"
            used[cell] += 1
            q = sim.model.joint(f"{token}_free").qposadr[0]
            sim.data.qpos[q : q + 3] = (T_GRID @ [*layout.cell(LAYOUT, i % 7, i // 7), 1.0])[:3]
            sim.data.qpos[q + 3 : q + 7] = quat
    mujoco.mj_forward(sim.model, sim.data)
    for _ in range(500):  # 1 s: the tokens settle on each other and against a plate
        mujoco.mj_step(sim.model, sim.data)
    assert Connect4Sim(sim, LAYOUT, None).board() == cells, name
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    renderer.update_scene(sim.data, camera=sim.camera_name)
    cv2.imwrite(str(OUT / f"{name}.png"), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    renderer.close()
    sim.disconnect()
    return K


if __name__ == "__main__":
    config = Path(get_package_share_directory("so101_bringup")) / "config" / "sim_scene.yaml"
    camera = yaml.safe_load(config.read_text())["camera"]
    reference = {"boards": boards(np.random.default_rng(81))}
    for name, cells in reference["boards"].items():
        K = render(name, cells, camera)
    reference["K"] = np.asarray(K).round(4).tolist()
    (OUT / "reference.json").write_text(json.dumps(reference, indent=1) + "\n")
    print("rendered", sorted(reference["boards"]))
