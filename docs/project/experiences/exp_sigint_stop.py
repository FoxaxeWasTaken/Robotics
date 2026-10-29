"""MES-44, MES-50: a SIGINT at a random moment, against two ways for a node to stop (#158, #177).

- spin: rclpy.spin() and its signal handlers, which raise KeyboardInterrupt at any bytecode (the driver before
  #158). Landing while the executor builds a Task, it leaves a half-built one: "Exception ignored in
  Future.__del__ ... 'Task' object has no attribute '_exception'" at exit.
- event: rclpy's handlers off, SIGINT sets an event that ends a spin_once() loop (the driver since #158).

With --perception, the installed perception node instead of the test node (#177), fed 30 Hz images of a
red square and camera_info, so that it takes and processes messages while it stops; its log line "ready"
is the start.

Each run starts a node with `timers` 1 kHz timers (more timers, more Tasks built, a wider window), sends
SIGINT 0.05 to 0.35 s after it is ready, and reads its stderr. In the control image, with ROS sourced:

Usage: python3 exp_sigint_stop.py [runs] [timers]
       python3 exp_sigint_stop.py --perception [runs]
"""

import random
import signal
import subprocess
import sys
import threading
import time


def child(mode, timers):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.ALL if mode == "spin" else SignalHandlerOptions.NO)
    node = rclpy.create_node("exp_sigint_stop")
    for _ in range(timers):
        node.create_timer(0.001, lambda: None)
    stop = threading.Event()
    if mode == "event":
        signal.signal(signal.SIGINT, lambda *_: stop.set())
    print("ready", flush=True)
    try:
        if mode == "spin":
            rclpy.spin(node)
        else:
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            while not stop.is_set():
                executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def feed():
    """The driver's camera topics at 30 Hz: a red square on grey, which perception detects."""
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node("exp_sigint_feed")
    images = node.create_publisher(Image, "/external_cam/image_raw", 2)
    infos = node.create_publisher(CameraInfo, "/external_cam/camera_info", 2)
    rgb = bytearray([128]) * (3 * 640 * 480)
    for row in range(220, 260):
        rgb[3 * (640 * row + 300) : 3 * (640 * row + 340)] = bytes([200, 20, 20]) * 40
    image = Image(height=480, width=640, encoding="rgb8", step=3 * 640, data=bytes(rgb))
    info = CameraInfo(width=640, height=480, k=[500.0, 0.0, 319.5, 0.0, 500.0, 239.5, 0.0, 0.0, 1.0])

    def publish():
        images.publish(image)
        infos.publish(info)

    node.create_timer(1 / 30, publish)
    rclpy.spin(node)


def stop_once(command):
    """'task' (the half-built Task traceback), 'other' (another error or a non-zero exit), 'hung' or 'clean'."""
    proc = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL),  # a background shell ignores it
    )
    for line in proc.stdout:
        if "ready" in line:  # the handlers are in place
            break
    time.sleep(random.uniform(0.05, 0.35))
    proc.send_signal(signal.SIGINT)
    try:
        out, _ = proc.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()
        return "hung"
    if "Future.__del__" in out:
        return "task"
    return "other" if "Traceback" in out or "Error" in out or proc.returncode else "clean"


def report(label, command, runs):
    results = [stop_once(command) for _ in range(runs)]
    counts = {k: results.count(k) for k in ("task", "other", "hung", "clean")}
    print(f"{label}: {counts} in {runs} stops", flush=True)


if __name__ == "__main__":
    if sys.argv[1:2] == ["--child"]:
        child(sys.argv[2], int(sys.argv[3]))
        sys.exit()
    if sys.argv[1:2] == ["--feed"]:
        feed()
        sys.exit()
    if sys.argv[1:2] == ["--perception"]:
        from ament_index_python.packages import get_package_prefix

        runs = int(sys.argv[2]) if len(sys.argv) > 2 else 200
        feeder = subprocess.Popen([sys.executable, __file__, "--feed"])
        try:
            perception = f"{get_package_prefix('so101_perception')}/lib/so101_perception/perception"
            report("perception", [perception], runs)
        finally:
            feeder.kill()
        sys.exit()
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 150
    timers = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    for mode in ("spin", "event"):
        report(f"{mode}, {timers} timers", [sys.executable, __file__, "--child", mode, str(timers)], runs)
