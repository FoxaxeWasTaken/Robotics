"""Driver control period with the sim camera thread: does rendering delay the 50 Hz tick? Feeds MES-34.

Runs the real DriverNode (sim) in-process for 30 s and timestamps every control tick (time.monotonic),
camera thread on, then off (camera loop replaced by a no-op), with one image subscriber in another process
(`ros2 topic hz`, as perception will be). Done-when of ticket 1.2a: p99 <= 25 ms.
In the control container, workspace built: python3 docs/project/experiences/exp_driver_jitter.py
"""
import subprocess
import time

import numpy as np
import rclpy
from so101_driver.driver_node import DriverNode

DURATION = 30.0  # s per run
control_step = DriverNode._control_step


def run(camera):
    ticks, frames = [], []

    def timed_step(self):
        ticks.append(time.monotonic())
        control_step(self)

    DriverNode._control_step = timed_step  # patched before the node binds it to its timer
    if not camera:
        DriverNode._camera_loop = lambda self: None
    rclpy.init()
    node = DriverNode()
    publish = node._pub_image.publish
    node._pub_image.publish = lambda msg: (frames.append(time.monotonic()), publish(msg))
    subscriber = subprocess.Popen(
        ["ros2", "topic", "hz", "/external_cam/image_raw"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    end = time.monotonic() + DURATION
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    subscriber.terminate()
    node.destroy_node()
    rclpy.shutdown()
    periods = np.diff(ticks) * 1e3
    steady = [t for t in frames if t > ticks[0] + 5.0]  # after the GL start-up (first frames slower)
    image_hz = (len(steady) - 1) / (steady[-1] - steady[0]) if len(steady) > 1 else 0.0
    print(
        f"camera {'on ' if camera else 'off'}: {len(periods)} control periods, "
        f"p50 {np.percentile(periods, 50):.1f} ms, p99 {np.percentile(periods, 99):.1f} ms, "
        f"max {periods.max():.1f} ms; image {image_hz:.1f} Hz"
    )


run(camera=True)
run(camera=False)
