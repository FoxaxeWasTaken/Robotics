"""Serial bus errors (docs/project/03 §8): the driver retries 3 control ticks, then stops /joint_states (the
brain stops on freshness) and still exits cleanly. Faults injected into the sim backend."""

import pytest
import rclpy

from so101_driver.driver_node import DriverNode


def fail(*_):
    raise ConnectionError("injected: there is no status packet")


@pytest.fixture
def driver(request):
    rclpy.init(args=getattr(request, "param", None))  # ROS arguments, from an indirect parametrize
    node = DriverNode()
    node.published = []
    node._pub_joint_states.publish = node.published.append  # spy: what reaches /joint_states
    yield node
    rclpy.shutdown()


def failed_ticks(driver, count):
    read = driver._robot.get_observation
    driver._robot.get_observation = fail
    for _ in range(count):
        driver._control_step()
    driver._robot.get_observation = read


def test_three_failed_ticks_are_retried_then_the_driver_carries_on(driver):
    failed_ticks(driver, 3)
    driver._control_step()
    driver._publish_joint_states()
    assert len(driver.published) == 1
    driver.destroy_node()


def test_a_fourth_failure_stops_joint_states_and_the_driver_still_exits(driver):
    failed_ticks(driver, 4)
    driver._control_step()  # the bus is back: too late, the driver must be restarted
    driver._publish_joint_states()
    assert driver.published == []
    driver._robot.send_action = driver._robot.get_observation = driver._robot.disconnect = fail
    driver.destroy_node()  # no shutdown move on a lost bus, and a failed disconnect does not raise


@pytest.mark.parametrize("driver", [["--ros-args", "-p", "bus_retries:=1"]], indirect=True)
def test_the_retries_are_a_parameter(driver):
    failed_ticks(driver, 2)
    driver._publish_joint_states()
    assert driver.published == []
    driver.destroy_node()
