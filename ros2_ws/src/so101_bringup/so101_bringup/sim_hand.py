"""sim_hand.py - the simulated person's hand (docs/project/phase2/P2-02 §7): a token of the
person's color into a column, one of the robot's between its jaws (the layout's token.robot_color), the grid
cleared, over the driver's /sim/connect4/* services.

Sim only, as those services: so101_connect4 names none of them (CI guard), its client takes the hand as an
object. The console (connect4_play) and the game bench (evaluate --games) share this one.
"""

import rclpy
from std_srvs.srv import Trigger

from so101_connect4.client import call
from so101_interfaces.srv import DropToken


class SimHand:
    """Each method returns the driver's (success, message)."""

    def __init__(self, node, spin_once=None):
        self._spin_once = spin_once or (lambda: rclpy.spin_once(node, timeout_sec=0.05))
        self._drop = node.create_client(DropToken, "/sim/connect4/drop_token")
        self._give = node.create_client(Trigger, "/sim/connect4/give_token")
        self._reset = node.create_client(Trigger, "/sim/connect4/reset_board")

    def drop_token(self, column) -> tuple:
        request = DropToken.Request(column=column, color=DropToken.Request.HUMAN)
        return call(self._spin_once, self._drop, request)

    def give_token(self) -> tuple:
        return call(self._spin_once, self._give)

    def reset_board(self) -> tuple:
        return call(self._spin_once, self._reset)
