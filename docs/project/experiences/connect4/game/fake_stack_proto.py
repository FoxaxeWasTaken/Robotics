"""The fake world of test_turns.py behind ROS: the board reader, the brain and the sim services, for game_node_proto.py.
/end_effector_pose starts late on purpose: game_node must refuse a new game until then (the "Brain node ready" trap)."""

import json
import random
import signal
import sys
import threading
import time

import rclpy
import rclpy.node
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, ".")
from game import EMPTY
from test_turns import World

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
POSE_AFTER_S = 3.0


class Fake(rclpy.node.Node):
    def __init__(self):
        super().__init__("fake_stack")
        self.w = World(random.Random(0), {})
        self.start = time.monotonic()
        self.pub_board = self.create_publisher(String, "/connect4/board", 1)
        self.pub_brain = self.create_publisher(String, "/brain_state", LATCHED)
        self.pub_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self.create_service(ChangeState, "/brain/drop", self.drop)
        self.create_service(Trigger, "/brain/stop", self.stop)
        self.create_service(ChangeState, "/sim/connect4/drop_token", self.drop_token)
        self.create_service(Trigger, "/sim/connect4/give_token", self.give)
        self.create_service(Trigger, "/sim/connect4/reset", self.reset)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.sticky)
        self.seen = None

    def now(self):
        return time.monotonic() - self.start

    def drop(self, request, response):
        column = request.transition.id
        self.w.brain_drop(self.now(), None if column == 255 else column)
        response.success = self.w.reply[0]
        self.w.reply = None
        self.say()
        return response

    def stop(self, request, response):
        self.w.brain_stop()
        self.say()
        response.success = True
        return response

    def drop_token(self, request, response):
        self.w.drop_token(self.now(), request.transition.id, 1)
        response.success = True
        return response

    def give(self, request, response):
        self.w.held = True
        response.success = True
        return response

    def reset(self, request, response):
        self.w.grid = [EMPTY] * 42
        self.w.flying, self.w.held = [], False
        response.success = True
        return response

    def say(self):
        labels, self.w.labels = self.w.labels, []
        for label in labels:
            self.pub_brain.publish(String(data=label))

    def sticky(self):
        self.pub_brain.publish(String(data=self.w.last_label))

    def tick(self):
        board, _, _ = self.w.tick(self.now())
        self.w.labels and self.say()
        if self.now() > POSE_AFTER_S:
            self.pub_pose.publish(PoseStamped())
        if board is not None and board is not self.seen:
            self.seen = board
            data = {"cells": list(board.cells), "valid": board.valid, "reason": board.reason}
            self.pub_board.publish(String(data=json.dumps(data)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    executor = SingleThreadedExecutor()
    executor.add_node(Fake())
    while not stop.is_set():
        executor.spin_once(timeout_sec=0.1)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
