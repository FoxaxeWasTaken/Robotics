"""The real driver on the Connect Four scene (#79, phase2/P2-02 §7): a whole game of tokens falls while the
arm moves and the camera renders at 640x480; the 50 Hz tick and the image rate hold, and every token ends in
its cell. In-process, as docs/project/experiences/exp_driver_jitter.py [MES-34]: the tick is stamped where the
timer calls it. The tokens are placed as /sim/connect4/drop_token will (#80): MjData written under the
driver's lock. Ground truth read to score (C-06)."""

import os
import subprocess
import time

import numpy as np
import rclpy
from test_connect4_scene import CAMERA, GRID, LAYOUT, cell_of, place

from so101_bringup import sim_scene
from so101_driver.driver_node import DriverNode

SECONDS = 60.0
FIRST_DROP, DROP_EVERY = 5.0, 0.75  # s: after the GL start-up, then a token each 0.75 s: 42 in 31.5 s
# The 4-vCPU CI runner renders with 2 threads under coverage: 4.9 Hz at 320x240 where a developer machine
# gives 10 [MES-34]. There the test only says the driver is alive and keeps real time; the targets are a
# developer machine's.
SMALL_MACHINE = (os.cpu_count() or 1) <= 4
MAX_P99_MS, MIN_IMAGE_HZ = (40.0, 1.0) if SMALL_MACHINE else (25.0, 3.0)


def test_a_game_of_42_tokens_falls_with_the_tick_and_the_image_rate_kept(tmp_path, monkeypatch):
    scene = sim_scene.generate(tmp_path, camera=CAMERA, connect4=LAYOUT)
    ticks, frames = [], []
    control_step = DriverNode._control_step

    def stamped_step(self):
        ticks.append(time.monotonic())
        control_step(self)

    monkeypatch.setattr(DriverNode, "_control_step", stamped_step)  # before the node binds it to its timer
    parameters = [f"scene_path:={scene}", "camera_width:=640", "camera_height:=480"]
    rclpy.init(args=["--ros-args", *(arg for p in parameters for arg in ("-p", p))])
    reader = subprocess.Popen(  # a subscriber in another process, as the board reader will be
        ["ros2", "topic", "hz", "/external_cam/image_raw"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    node = DriverNode()
    try:
        publish = node._pub_image.publish
        node._pub_image.publish = lambda msg: (frames.append(time.monotonic()), publish(msg))
        tokens = [f"token_{'red' if i % 2 else 'yellow'}_{i // 2}" for i in range(42)]
        start, dropped = time.monotonic(), 0
        while (now := time.monotonic() - start) < SECONDS:
            node._goal["shoulder_pan"] = -1.0 + 0.5 * np.sin(now)  # the arm moves, away from the grid
            node._goal["wrist_flex"] = 0.8 + 0.3 * np.sin(1.3 * now)
            if dropped < len(tokens) and now > FIRST_DROP + DROP_EVERY * dropped:
                with node._backend_lock:
                    place(node._robot, tokens[dropped], dropped % GRID["columns"])
                dropped += 1
            rclpy.spin_once(node, timeout_sec=0.02)
        with node._backend_lock:
            cells = [cell_of(node._robot, token) for token in tokens]
    finally:
        reader.terminate()
        node.destroy_node()
        rclpy.shutdown()
    periods = np.diff(ticks[50:]) * 1e3  # after the first second
    p50, p99 = np.percentile(periods, [50, 99])
    steady = [t for t in frames if t > start + FIRST_DROP]
    image_hz = (len(steady) - 1) / (steady[-1] - steady[0]) if len(steady) > 1 else 0.0
    in_cell = sum(
        abs(column - i % GRID["columns"]) < 0.1 and abs(row - i // GRID["columns"]) < 0.1
        for i, (column, row, _) in enumerate(cells)
    )
    print(
        f"connect4 driver, {SECONDS:.0f} s, {os.cpu_count()} CPUs: {len(periods)} ticks, p50 {p50:.1f} ms, "
        f"p99 {p99:.1f} ms, max {periods.max():.1f} ms; image {image_hz:.1f} Hz at 640x480; "
        f"{in_cell}/{len(tokens)} tokens in their cell"
    )
    assert in_cell == len(tokens), [(i, np.round(c, 2).tolist()) for i, c in enumerate(cells)]
    assert 19.0 < p50 < 21.0, "the sim no longer runs in real time"
    assert p99 <= MAX_P99_MS
    assert image_hz >= MIN_IMAGE_HZ
