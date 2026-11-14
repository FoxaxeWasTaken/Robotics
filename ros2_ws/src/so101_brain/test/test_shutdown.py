"""Unit test: a destroyed brain leaves no thread of its planning pool behind (#272).

At the interpreter's exit, Python 3.12 wakes each process pool's manager thread without the pool's lock. A
pool shut down without waiting may be closing its pipe at that moment: the write fails and the brain's stop
prints `OSError: [Errno 9] Bad file descriptor` under a traceback, which test_brain_launch.py refuses (once
in the CI, run 43613156760).
"""

import threading

import rclpy

from so101_brain.brain_node import BrainNode


def test_a_destroyed_brain_has_joined_the_threads_of_its_planning_pool():
    before = set(threading.enumerate())
    rclpy.init()
    try:
        BrainNode().destroy_node()
        left = [thread.name for thread in threading.enumerate() if thread not in before]
    finally:
        rclpy.shutdown()
    assert left == []
