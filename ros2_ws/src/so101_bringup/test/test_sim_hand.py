"""sim_hand.py (#232, #87): the simulated person's hand calls the driver's /sim/connect4/* services, here
served by a fake driver in this process. The real services are test_connect4_driver.py's."""

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

from so101_bringup.sim_hand import SimHand
from so101_interfaces.srv import DropToken


@pytest.fixture
def driver():
    rclpy.init()
    node = rclpy.create_node("fake_driver")
    node.calls, node.reply = [], (True, "done")

    def serve(name):
        def served(request, response):
            node.calls.append((name, request.column, request.color) if name == "drop_token" else (name,))
            response.success, response.message = node.reply
            return response

        return served

    node.create_service(DropToken, "/sim/connect4/drop_token", serve("drop_token"))
    node.create_service(Trigger, "/sim/connect4/give_token", serve("give_token"))
    node.create_service(Trigger, "/sim/connect4/reset_board", serve("reset_board"))
    node.create_service(Trigger, "/sim/connect4/empty_dispenser", serve("empty_dispenser"))
    node.create_service(Trigger, "/sim/connect4/fill_dispenser", serve("fill_dispenser"))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node.hand = SimHand(node, spin_once=lambda: executor.spin_once(timeout_sec=0.01))
    yield node
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def test_the_hand_drops_a_red_token_gives_a_yellow_one_and_clears_the_grid(driver):
    assert driver.hand.drop_token(4) == (True, "done")
    assert driver.hand.give_token() == (True, "done")
    assert driver.hand.reset_board() == (True, "done")
    assert driver.calls == [("drop_token", 4, DropToken.Request.HUMAN), ("give_token",), ("reset_board",)]


def test_a_refusal_of_the_driver_comes_back_with_its_message(driver):
    driver.reply = (False, "column 4 is full")
    assert driver.hand.drop_token(4) == (False, "column 4 is full")


def test_the_hand_empties_the_dispenser_and_fills_it(driver):
    assert driver.hand.empty_dispenser() == (True, "done")
    assert driver.hand.fill_dispenser() == (True, "done")
    assert driver.calls == [("empty_dispenser",), ("fill_dispenser",)]
