"""The real driver on the Connect Four scene (#79, phase2/P2-02 §7): a whole game of tokens falls while the
arm moves and the camera renders at 640x480; the 50 Hz tick and the image rate hold, and every token ends in
its cell. In-process, as docs/project/experiences/exp_driver_jitter.py [MES-34]: the tick is stamped where the
timer calls it. The tokens come from the driver's own /sim/connect4/drop_token (#80), and a token given by
/sim/connect4/give_token stays in the closed jaws while the arm moves. Ground truth read to score (C-06)."""

import os
import subprocess
import time

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
from test_connect4_scene import CAMERA, GRID, HANDOVER, LAYOUT

from so101_bringup import sim_scene
from so101_common.robot import ARM_JOINT_NAMES
from so101_driver.connect4_sim import HUMAN, ROBOT
from so101_driver.driver_node import DriverNode
from so101_interfaces.srv import DropToken

LAYOUT_FILE = os.path.join(get_package_share_directory("so101_connect4"), "config", "connect4_grid.yaml")

SECONDS = 60.0
FIRST_DROP, DROP_EVERY = 5.0, 0.75  # s: after the GL start-up, then a token each 0.75 s: 42 in 31.5 s
# The 4-vCPU CI runner renders with 2 threads under coverage: 4.9 Hz at 320x240 where a developer machine
# gives 10 [MES-34]. There the test only says the driver is alive and keeps real time; the targets are a
# developer machine's.
SMALL_MACHINE = (os.cpu_count() or 1) <= 4
MAX_P99_MS, MIN_IMAGE_HZ = (40.0, 1.0) if SMALL_MACHINE else (25.0, 3.0)


@pytest.fixture
def game(tmp_path, monkeypatch):
    """The driver in game mode on the game's scene, as sim.launch.py connect4:=true starts it; game.ticks:
    when each control step began."""
    ticks = []
    control_step = DriverNode._control_step

    def stamped_step(self):
        ticks.append(time.monotonic())
        control_step(self)

    monkeypatch.setattr(DriverNode, "_control_step", stamped_step)  # before the node binds it to its timer
    scene = sim_scene.generate(tmp_path, camera=CAMERA, connect4=LAYOUT)
    parameters = [
        f"scene_path:={scene}",
        f"connect4_layout:={LAYOUT_FILE}",
        "camera_width:=640",
        "camera_height:=480",
    ]
    rclpy.init(args=["--ros-args", *(arg for p in parameters for arg in ("-p", p))])
    node = DriverNode()
    node.ticks = ticks
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_a_given_token_stays_in_the_closed_jaws_while_the_arm_moves(game):
    """The hold through the driver's own tick (P2-R-15): give, close, then swing the arm at its top speed."""

    def run(seconds, arm, gripper):
        game._goal.update(zip(ARM_JOINT_NAMES, arm, strict=True), gripper=gripper)
        for _ in range(round(seconds * 50)):
            game._control_step()

    def in_gripper():
        data = game._robot.data
        R = data.body("gripper").xmat.reshape(3, 3)
        return R.T @ (data.body("token_yellow_0").xpos - data.site("gripperframe").xpos)

    run(3.0, HANDOVER, 0.0)
    empty = game._effort[-1]  # the gripper's closing effort, as /joint_states gives it (03 §5)
    run(1.0, HANDOVER, 60.0)
    reply = game._srv_give_token(Trigger.Request(), Trigger.Response())
    assert reply.success, reply.message
    run(1.0, HANDOVER, 0.0)
    assert game._connect4.floating is None  # both jaws touch it: its weight is back
    on_token, jaws = game._effort[-1], game._obs["gripper.pos"]
    held = in_gripper()
    above_the_grid = [0.7, -0.3, 0.2, 1.2, 1.6195]  # towards the grid's azimuth, the wrist rolled as a drop
    run(2.5, above_the_grid, 0.0)
    run(2.5, HANDOVER, 0.0)
    slip = np.linalg.norm(in_gripper() - held) * 1e3
    print(
        f"token held: closing effort {on_token:.2f} on it, {empty:.2f} empty; jaws at {jaws:.2f} %; "
        f"slip {slip:.1f} mm after two moves at 1.5 rad/s"
    )
    assert slip < 4.0  # MES-63: under 4 mm up to 1.5 rad/s
    # What tells a held token from none is the effort, as on the cube [MES-10]: the jaws end within 0.2 % of
    # an empty close, the token sits where they do not meet [MES-67].
    assert empty < 0.05
    assert on_token > 0.25


def test_a_game_of_42_tokens_falls_with_the_tick_and_the_image_rate_kept(game):
    node, ticks, frames = game, game.ticks, []
    reader = subprocess.Popen(  # a subscriber in another process, as the board reader will be
        ["ros2", "topic", "hz", "/external_cam/image_raw"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        publish = node._pub_image.publish
        node._pub_image.publish = lambda msg: (frames.append(time.monotonic()), publish(msg))
        colors = [HUMAN if i % 2 else ROBOT for i in range(GRID["columns"] * GRID["rows"])]
        start, dropped = time.monotonic(), 0
        while (now := time.monotonic() - start) < SECONDS:
            node._goal["shoulder_pan"] = -1.0 + 0.5 * np.sin(now)  # the arm moves, away from the grid
            node._goal["wrist_flex"] = 0.8 + 0.3 * np.sin(1.3 * now)
            if dropped < len(colors) and now > FIRST_DROP + DROP_EVERY * dropped:
                request = DropToken.Request(column=dropped % GRID["columns"], color=colors[dropped])
                reply = node._srv_drop_token(request, DropToken.Response())
                assert reply.success, (dropped, reply.message)
                dropped += 1
            rclpy.spin_once(node, timeout_sec=0.02)
        with node._backend_lock:
            cells = node._connect4.board()
    finally:
        reader.terminate()
    periods = np.diff(ticks[50:]) * 1e3  # after the first second
    p50, p99 = np.percentile(periods, [50, 99])
    steady = [t for t in frames if t > start + FIRST_DROP]
    image_hz = (len(steady) - 1) / (steady[-1] - steady[0]) if len(steady) > 1 else 0.0
    in_cell = sum(cell == color for cell, color in zip(cells, colors, strict=True))  # dropped row by row
    print(
        f"connect4 driver, {SECONDS:.0f} s, {os.cpu_count()} CPUs: {len(periods)} ticks, p50 {p50:.1f} ms, "
        f"p99 {p99:.1f} ms, max {periods.max():.1f} ms; image {image_hz:.1f} Hz at 640x480; "
        f"{in_cell}/{len(colors)} tokens in their cell"
    )
    assert in_cell == len(colors), cells
    assert 19.0 < p50 < 21.0, "the sim no longer runs in real time"
    assert p99 <= MAX_P99_MS
    assert image_hz >= MIN_IMAGE_HZ
