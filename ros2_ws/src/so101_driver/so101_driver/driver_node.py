#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.

Publishes /joint_states (rad, 30 Hz) and applies /joint_command (rad, gripper in %)
through a speed limiter at 50 Hz (docs/project/03 §3-§5).
"""

from pathlib import Path

import rclpy
import rclpy.node
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from so101_sim import SO101Sim

from so101_common.robot import ARM_JOINT_NAMES, JOINT_NAMES, pct_to_rad
from so101_driver.limiter import step_towards

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
DEFAULT_PORT = "/dev/ttyACM0"

CONTROL_HZ = 50.0
JOINT_STATE_HZ = 30.0
MAX_JOINT_VELOCITY = 1.5  # rad/s, speed limiter (ADR-023); becomes a parameter at step 1.1
MAX_GRIPPER_VELOCITY = 200.0  # %/s


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")

        self._obs = self._robot.get_observation()
        self._obs_stamp = self.get_clock().now().to_msg()  # /joint_states stamp = read time (03 §5)
        self._goal = {name: self._obs[f"{name}.pos"] for name in JOINT_NAMES}  # startup pose: no jump to 0
        self._setpoint = dict(self._goal)

        self._pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self._cb_joint_command, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)
        self.create_timer(1.0 / JOINT_STATE_HZ, self._publish_joint_states)

        self.get_logger().info("Driver node ready.")

    def _connect_arm(self, port: str = DEFAULT_PORT) -> SO101Sim | SO101Follower:
        """Creates and connects the backend chosen by use_sim (provided)."""
        if self._use_sim:
            robot = SO101Sim()
            robot.connect()
            return robot

        if not CALIBRATION_FILE.is_file():
            raise RuntimeError(
                f"No calibration file at {CALIBRATION_FILE}: set CALIBRATION_FILE in docker/.env"
            )
        config = SO101FollowerConfig(
            port=port,
            id=CALIBRATION_FILE.stem,
            calibration_dir=CALIBRATION_FILE.parent,
            use_radians=True,
        )
        robot = SO101Follower(config)
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            robot.disconnect()
            raise RuntimeError(
                f"{CALIBRATION_FILE} does not match the motors: wrong file for this arm? "
                "Re-run lerobot-calibrate on the host."
            )
        robot.bus.disable_torque()  # Comment out this line to control the robot.
        self.get_logger().info(
            "Torque disabled: arm can be moved freely by hand. Remove this part to control the arm."
        )
        return robot

    def destroy_node(self):
        """Disconnects the backend (provided). On the real arm this disables the torque."""
        if self._robot.is_connected:
            self._robot.disconnect()
            self.get_logger().info("Robot disconnected.")
        super().destroy_node()

    def _cb_joint_command(self, msg: JointState):
        """Stores the goal only (the control timer applies it). No name = JOINT_NAMES order."""
        names = list(msg.name) or list(JOINT_NAMES)
        if len(names) != len(msg.position):
            self.get_logger().warning(
                f"/joint_command ignored: {len(names)} names, {len(msg.position)} positions",
                throttle_duration_sec=1.0,
            )
            return
        for name, value in zip(names, msg.position, strict=True):
            if name in self._goal:  # unknown names are ignored; other joints keep their goal
                self._goal[name] = float(value)

    def _control_step(self):
        """50 Hz: setpoint towards the goal (speed limiter), send it, read the state back."""
        self._setpoint = step_towards(
            self._setpoint, self._goal, 1.0 / CONTROL_HZ, MAX_JOINT_VELOCITY, MAX_GRIPPER_VELOCITY
        )
        self._robot.send_action({f"{name}.pos": value for name, value in self._setpoint.items()})
        self._obs = self._robot.get_observation()
        self._obs_stamp = self.get_clock().now().to_msg()

    def _publish_joint_states(self):
        """30 Hz: last observation, arm in rad, gripper converted % -> rad (what RSP expects)."""
        msg = JointState()
        msg.header.stamp = self._obs_stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [
            *(self._obs[f"{name}.pos"] for name in ARM_JOINT_NAMES),
            pct_to_rad(self._obs["gripper.pos"]),
        ]
        self._pub_joint_states.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
