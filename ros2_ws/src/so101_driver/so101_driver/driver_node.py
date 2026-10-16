#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.

Publishes /joint_states (rad, 30 Hz) and applies /joint_command (rad, gripper in %)
through a speed limiter at 50 Hz; in sim, publishes the camera at 10 Hz from a dedicated
thread (docs/project/03 §3-§5).
"""

import os
import threading
import time
from pathlib import Path

import mujoco
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image, JointState
from so101_sim import SO101Sim

from so101_common.robot import ARM_JOINT_NAMES, CAMERA_FRAME, JOINT_NAMES, pct_to_rad
from so101_driver.command import apply_command
from so101_driver.limiter import step_towards

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
DEFAULT_PORT = "/dev/ttyACM0"

# Startup-only parameters (docs/project/01 §3.3), values in so101_bringup/config/driver.yaml:
# name: (default, description, min, max). The image size is bounded by the scene's offscreen buffer.
PARAMETERS = {
    "control_rate_hz": (50.0, "Control tick: speed limiter, send_action, get_observation", 10.0, 100.0),
    "joint_state_rate_hz": (30.0, "/joint_states publication (>= 20 Hz imposed, C-11)", 20.0, 100.0),
    "camera_rate_hz": (10.0, "Sim camera /external_cam/image_raw", 1.0, 30.0),
    "max_joint_velocity": (1.5, "Speed limiter, arm joints, rad/s (ADR-023)", 0.05, 3.0),
    "max_gripper_velocity": (200.0, "Speed limiter, gripper, %/s (ADR-023)", 10.0, 1000.0),
    "camera_width": (320, "Sim camera width, px (640 for calibration, ADR-013)", 16, 1280),
    "camera_height": (240, "Sim camera height, px (480 for calibration, ADR-013)", 16, 720),
}

# OSMesa's llvmpipe renders with one thread per core by default; they starve the 50 Hz control tick
# (p99 up to 25.4 ms). Half the cores keep it at 20.4 ms with the image at 10 Hz [MES-34]. A value set in the
# environment wins. Set at import, before any thread exists: setenv is not thread-safe.
os.environ.setdefault("LP_NUM_THREADS", str(max(1, (os.cpu_count() or 2) // 2)))


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")
        self._p = {name: self._declare(name, *spec) for name, spec in PARAMETERS.items()}

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")
        if self._use_sim:  # SO101Sim is built by the provided _connect_arm (ADR-032): image size set here
            self._robot.camera_width = self._p["camera_width"]
            self._robot.camera_height = self._p["camera_height"]
        self._backend_lock = threading.Lock()  # MuJoCo and the serial bus are not thread-safe (ADR-006)

        self._obs = self._robot.get_observation()
        self._obs_stamp = self.get_clock().now().to_msg()  # /joint_states stamp = read time (03 §5)
        self._goal = {name: self._obs[f"{name}.pos"] for name in JOINT_NAMES}  # startup pose: no jump to 0
        self._setpoint = dict(self._goal)

        self._pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self._cb_joint_command, 10)
        self.create_timer(1.0 / self._p["control_rate_hz"], self._control_step)
        self.create_timer(1.0 / self._p["joint_state_rate_hz"], self._publish_joint_states)

        self._pub_image = self.create_publisher(Image, "/external_cam/image_raw", 2)
        self._pub_camera_info = self.create_publisher(CameraInfo, "/external_cam/camera_info", 2)
        self._stop_camera = threading.Event()
        self._camera_thread = None
        if self._use_sim:  # the real camera comes at 6.2a
            self._camera_thread = threading.Thread(target=self._camera_loop, name="sim_camera", daemon=True)
            self._camera_thread.start()

        self.get_logger().info("Driver node ready.")

    def _declare(self, name, default, description, low, high):
        """Declares a read-only parameter with its range; an out-of-range value stops the node at startup."""
        if isinstance(default, int):
            limits = {"integer_range": [IntegerRange(from_value=low, to_value=high, step=1)]}
        else:
            limits = {"floating_point_range": [FloatingPointRange(from_value=low, to_value=high)]}
        descriptor = ParameterDescriptor(description=description, read_only=True, **limits)
        return self.declare_parameter(name, default, descriptor).value

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
        """Stops the camera thread, then disconnects the backend (provided).

        On the real arm, disconnecting disables the torque.
        """
        self._stop_camera.set()
        if self._camera_thread is not None:
            self._camera_thread.join()
        if self._robot.is_connected:
            self._robot.disconnect()
            self.get_logger().info("Robot disconnected.")
        super().destroy_node()

    def _cb_joint_command(self, msg: JointState):
        """Stores the goal only (the control timer applies it): clamped, or unchanged if rejected (03 §4)."""
        self._goal, reason = apply_command(self._goal, msg.name, msg.position)
        if reason:
            self.get_logger().warning(f"/joint_command rejected: {reason}", throttle_duration_sec=1.0)

    def _control_step(self):
        """50 Hz: setpoint towards the goal (speed limiter), send it, read the state back."""
        self._setpoint = step_towards(
            self._setpoint,
            self._goal,
            1.0 / self._p["control_rate_hz"],
            self._p["max_joint_velocity"],
            self._p["max_gripper_velocity"],
        )
        with self._backend_lock:
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

    def _camera_loop(self):
        """10 Hz, sim only: copy the state under the lock, render the copy outside it (ADR-006).

        Nothing renders the live MjData: the control timer keeps stepping it while a frame renders.
        """
        sim = self._robot
        state = mujoco.MjData(sim.model)
        # Created here: its OpenGL context is bound to this thread.
        renderer = mujoco.Renderer(sim.model, height=sim.camera_height, width=sim.camera_width)
        bridge = CvBridge()
        info = camera_info(sim.get_camera_intrinsics(), sim.camera_width, sim.camera_height)
        next_frame = time.monotonic()
        try:
            while not self._stop_camera.wait(max(0.0, next_frame - time.monotonic())):
                # A late frame shifts the schedule: no catch-up burst.
                next_frame = max(next_frame + 1.0 / self._p["camera_rate_hz"], time.monotonic())
                with self._backend_lock:
                    mujoco.mj_copyData(state, sim.model, sim.data)
                stamp = self.get_clock().now().to_msg()  # image stamp = copy time, like /joint_states
                renderer.update_scene(state, camera=sim.camera_name)
                msg = bridge.cv2_to_imgmsg(renderer.render(), encoding="rgb8")
                msg.header.stamp = stamp
                msg.header.frame_id = CAMERA_FRAME
                info.header = msg.header  # same stamp and frame_id: perception pairs them (C-13)
                try:
                    self._pub_image.publish(msg)
                    self._pub_camera_info.publish(info)
                except Exception:
                    if rclpy.ok():
                        raise
                    break  # SIGINT/SIGTERM shut the context down during the render: stop quietly
        finally:
            renderer.close()


def camera_info(K, width: int, height: int) -> CameraInfo:
    """CameraInfo of an ideal pinhole camera (sim): K, no distortion, P = [K | 0] (docs/project/03 §6)."""
    info = CameraInfo(width=width, height=height, distortion_model="plumb_bob")
    info.k = [float(v) for v in K.flatten()]
    info.d = [0.0] * 5
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [*info.k[0:3], 0.0, *info.k[3:6], 0.0, *info.k[6:9], 0.0]
    return info


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
        # SIGINT/SIGTERM shut the context down while a callback was publishing: a normal stop.
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
