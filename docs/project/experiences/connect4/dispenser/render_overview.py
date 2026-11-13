"""render_overview.py [azimuth elevation distance]: the README's picture of the game's scene, seen from a free
camera that holds the grid, the arm and the token dispenser (the scene camera cuts the dispenser's rails): a
game under way, the arm at HANDOVER. Writes /out/connect4_scene.png, 960x720.

    tools/so101 run --out docs/project/media \
        'python3 /repo/docs/project/experiences/connect4/dispenser/render_overview.py'
"""

import sys
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
from so101_driver.connect4_sim import HUMAN, ROBOT, Connect4Sim

AZIMUTH, ELEVATION, DISTANCE = (float(v) for v in (sys.argv[1:4] or (222.0, -24.0, 0.95)))
BRINGUP = Path(get_package_share_directory("so101_bringup")) / "config"
CAMERA = yaml.safe_load((BRINGUP / "sim_scene.yaml").read_text())["camera"]
HANDOVER = yaml.safe_load((BRINGUP / "brain.yaml").read_text())["so101_brain"]["ros__parameters"]["connect4"][
    "handover_pose"
]
LAYOUT = layout.load()
MOVES = [3, 3, 4, 2, 4, 4, 5, 1, 2, 3, 6, 0, 5]  # a game under way: the person first

sim = SO101Sim(scene_path=sim_scene.generate(Path(tempfile.mkdtemp()), camera=CAMERA, connect4=LAYOUT))
sim.connect()
model, data = sim.model, sim.data
model.vis.global_.offwidth, model.vis.global_.offheight = 960, 720
for joint, value in zip(ARM_JOINTS, HANDOVER, strict=False):
    data.qpos[model.joint(joint).qposadr[0]] = value
    data.ctrl[model.actuator(joint).id] = value
person = Connect4Sim(sim, LAYOUT, np.random.default_rng(0))


def run(seconds):
    for step in range(round(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
        if step % 10 == 9:
            person.tick()


run(1.0)
for i, column in enumerate(MOVES):
    assert person.drop_token(column, ROBOT if i % 2 else HUMAN)[0]
    run(0.8)
run(1.0)
view = mujoco.MjvCamera()
view.lookat[:] = [0.22, 0.08, 0.05]
view.azimuth, view.elevation, view.distance = AZIMUTH, ELEVATION, DISTANCE
renderer = mujoco.Renderer(model, height=720, width=960)
renderer.update_scene(data, camera=view)
cv2.imwrite("/out/connect4_scene.png", cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
print("wrote /out/connect4_scene.png", AZIMUTH, ELEVATION, DISTANCE)
