"""Late ticks (#176): a control step or a /joint_states publication that comes well after its period shows
as a warning in the driver's log, before the brain stops on stale /joint_states (0.5 s)."""

import time
from types import SimpleNamespace

import pytest
import rclpy

from so101_driver.driver_node import DriverNode
from so101_driver.late import LATE_WARN_S, TickClock


def test_the_first_tick_is_never_late():
    assert TickClock(0.02).late(100.0) == 0.0


def test_a_tick_on_time_or_early_is_not_late():
    ticks = TickClock(0.02)
    ticks.late(100.0)
    assert ticks.late(100.02) == 0.0
    assert ticks.late(100.03) == 0.0  # 10 ms early


def test_lateness_is_the_gap_beyond_one_period():
    ticks = TickClock(0.02)
    ticks.late(100.0)
    assert ticks.late(100.62) == pytest.approx(0.6)


def test_the_warning_bound_is_below_the_brains_stale_bound():
    assert LATE_WARN_S < 0.5  # brain.yaml max_joint_state_age_s


@pytest.fixture
def driver():
    rclpy.init()
    node = DriverNode()
    node.warnings = []
    log = node.get_logger()
    node.get_logger = lambda: SimpleNamespace(warning=node.warnings.append, info=log.info, error=log.error)
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize("callback", ["_control_step", "_publish_joint_states"])
def test_a_stalled_callback_is_logged(driver, callback):
    getattr(driver, callback)()
    time.sleep(LATE_WARN_S + 0.1)
    getattr(driver, callback)()
    assert len(driver.warnings) == 1, driver.warnings
    assert "late" in driver.warnings[0]


def test_callbacks_on_time_log_nothing(driver):
    for _ in range(3):
        driver._control_step()
        driver._publish_joint_states()
    assert driver.warnings == []
