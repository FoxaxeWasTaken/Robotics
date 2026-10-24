"""MES-44: a SIGINT at a random moment, against two ways for a node to stop (#158).

- spin: rclpy.spin() and its signal handlers, which raise KeyboardInterrupt at any bytecode (the driver before
  #158). Landing while the executor builds a Task, it leaves a half-built one: "Exception ignored in
  Future.__del__ ... 'Task' object has no attribute '_exception'" at exit.
- event: rclpy's handlers off, SIGINT sets an event that ends a spin_once() loop (the driver since #158).

Each run starts a node with `timers` 1 kHz timers (more timers, more Tasks built, a wider window), sends
SIGINT 0.05 to 0.35 s after it is ready, and reads its stderr. In the control image, with ROS sourced:

Usage: python3 exp_sigint_stop.py [runs] [timers]
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


def stop_once(mode, timers):
    """'task' (the half-built Task traceback), 'other' (another error), 'hung' or 'clean'."""
    proc = subprocess.Popen(
        [sys.executable, __file__, "--child", mode, str(timers)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL),  # a background shell ignores it
    )
    proc.stdout.readline()  # "ready": the handlers are in place
    time.sleep(random.uniform(0.05, 0.35))
    proc.send_signal(signal.SIGINT)
    try:
        _, err = proc.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()
        return "hung"
    if "Future.__del__" in err:
        return "task"
    return "other" if "Traceback" in err or "Error" in err else "clean"


if __name__ == "__main__":
    if sys.argv[1:2] == ["--child"]:
        child(sys.argv[2], int(sys.argv[3]))
        sys.exit()
    runs = int(sys.argv[1]) if len(sys.argv) > 1 else 150
    timers = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    for mode in ("spin", "event"):
        results = [stop_once(mode, timers) for _ in range(runs)]
        counts = {k: results.count(k) for k in ("task", "other", "hung", "clean")}
        print(f"{mode}, {timers} timers: {counts} in {runs} stops", flush=True)
