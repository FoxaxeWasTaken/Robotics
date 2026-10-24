#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.

Publishes /joint_states (rad, 30 Hz) and applies /joint_command (rad, gripper in %)
through a speed limiter at 50 Hz; publishes the camera at 10 Hz from a dedicated thread, the sim render
or a real webcam (camera_device) (docs/project/03 §3-§5); in sim, serves the evaluation tools:
/sim/randomize, ground truth (03 §7).
On shutdown, brings the arm slowly to its rest pose before disconnect() cuts the torque (03 §8).
"""

import os
import signal
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
import rclpy.node
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, JointState
from so101_sim import SO101Sim
from std_srvs.srv import Trigger

from so101_common.robot import ARM_JOINT_NAMES, BASE_FRAME, CAMERA_FRAME, JOINT_NAMES, pct_to_rad
from so101_driver.camera import opencv_intrinsics
from so101_driver.command import apply_command
from so101_driver.effort import normalized_effort
from so101_driver.limiter import max_relative_target, step_towards
from so101_driver.randomize import sample_box_xy
from so101_driver.shutdown import move_to

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
DEFAULT_PORT = "/dev/ttyACM0"

# Startup-only parameters (docs/project/01 §3.3), values in so101_bringup/config/driver.yaml:
# name: (default, description, min, max). The image size is bounded by the scene's offscreen buffer.
PARAMETERS = {
    "control_rate_hz": (50.0, "Control tick: speed limiter, send_action, get_observation", 10.0, 100.0),
    "joint_state_rate_hz": (30.0, "/joint_states publication (>= 20 Hz imposed, C-11)", 20.0, 100.0),
    "camera_rate_hz": (10.0, "/external_cam/image_raw, sim render or webcam", 1.0, 30.0),
    "max_joint_velocity": (1.5, "Speed limiter, arm joints, rad/s (ADR-023)", 0.05, 3.0),
    "max_gripper_velocity": (200.0, "Speed limiter, gripper, %/s (ADR-023)", 10.0, 1000.0),
    "camera_width": (320, "Image width, px: sim (640 for calibration, ADR-013) or webcam", 16, 1280),
    "camera_height": (240, "Image height, px: sim (480 for calibration, ADR-013) or webcam", 16, 720),
    "seed": (0, "Seed of /sim/randomize: cube and box positions (sim)", 0, 2**31 - 1),
    "max_relative_target_arm": (0.2, "Real arm: LeRobot max delta per command, arm joints, rad", 0.01, 1.0),
    "max_relative_target_gripper": (25.0, "Real arm: LeRobot max delta per command, gripper, %", 1.0, 100.0),
    "shutdown_velocity": (
        0.5,
        "Move to shutdown_pose before disconnect, rad/s, at most max_joint_velocity",
        0.1,
        1.5,
    ),
    "shutdown_tolerance": (0.05, "shutdown_pose reached when every arm joint is this close, rad", 0.005, 0.2),
    "bus_retries": (3, "Failed control ticks retried before the driver stops (03 §8)", 0, 20),
}
SHUTDOWN_POSE = [1.9, -1.7, 1.5, 0.7, 0.0]  # rad, ARM_JOINT_NAMES order: the brain's PARK (brain.yaml)
# ros2 launch sends SIGTERM 5 s after SIGINT, then SIGKILL: the move ends before. Signals only stop the spin.
SHUTDOWN_TIMEOUT = 4.0  # s
GROUND_TRUTH_RATE_HZ = 10.0
GROUND_TRUTH_BODIES = {"object": "pickup_object", "drop_box": "drop_box"}  # /sim/ground_truth/<key>: body
PARKED_BOX_XY = (2.0, 0.0)  # m: the box waits there while the randomized cube settles

# OSMesa's llvmpipe renders with one thread per core by default; they starve the 50 Hz control tick
# (p99 up to 25.4 ms). Half the cores keep it at 20.4 ms with the image at 10 Hz [MES-34]. A value set in the
# environment wins. Set at import, before any thread exists: setenv is not thread-safe.
os.environ.setdefault("LP_NUM_THREADS", str(max(1, (os.cpu_count() or 2) // 2)))


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter(
            "use_sim",
            True,
            ParameterDescriptor(description="Simulated backend or real arm (C-10)", read_only=True),
        ).value
        port: str = self.declare_parameter(
            "port",
            DEFAULT_PORT,
            ParameterDescriptor(description="Serial port of the real arm", read_only=True),
        ).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")
        self._p = {name: self._declare(name, *spec) for name, spec in PARAMETERS.items()}
        expose_ground_truth: bool = self.declare_parameter(
            "expose_ground_truth",
            False,
            ParameterDescriptor(
                description="Sim: publish /sim/ground_truth/*, bench only (C-06)", read_only=True
            ),
        ).value
        self._shutdown_pose = list(
            self.declare_parameter(
                "shutdown_pose",
                SHUTDOWN_POSE,
                ParameterDescriptor(description="Rest pose reached before disconnect, rad", read_only=True),
            ).value
        )
        camera_device: str = self.declare_parameter(
            "camera_device",
            "",
            ParameterDescriptor(
                description="Webcam (e.g. /dev/video0), also with the sim arm; empty: sim render or no image",
                read_only=True,
            ),
        ).value
        camera_info_file: str = self.declare_parameter(
            "camera_info_file",
            "",
            ParameterDescriptor(
                description="Webcam: ROS calibration YAML; empty: no camera_info", read_only=True
            ),
        ).value
        if len(self._shutdown_pose) != len(ARM_JOINT_NAMES):
            raise ValueError(f"shutdown_pose needs {len(ARM_JOINT_NAMES)} values, got {self._shutdown_pose}")
        # ROS ignores a key no node declares: a typo in driver_real.yaml would keep the sim's value (#57).
        if unknown := sorted(set(self._parameter_overrides) - set(self._parameters)):  # rclpy: no public API
            raise ValueError(f"Unknown parameters {unknown}: a typo in a config file? (01 §3.3)")

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")
        # SO101Sim is built by the provided _connect_arm (ADR-032): image size and seed are set here.
        if self._use_sim:
            self._robot.camera_width = self._p["camera_width"]
            self._robot.camera_height = self._p["camera_height"]
            # Two independent streams from one seed: the provided randomize_object() and the box sampler.
            # SO101Sim only takes a seed in its constructor, which _connect_arm calls without one.
            sim_seed, box_seed = np.random.SeedSequence(self._p["seed"]).spawn(2)
            self._robot._rng = np.random.default_rng(sim_seed)
            self._box_rng = np.random.default_rng(box_seed)
        else:  # after the provided _connect_arm (ADR-032): SO101Follower.send_action reads it at each call
            self._robot.config.max_relative_target = max_relative_target(
                self._p["max_relative_target_arm"], self._p["max_relative_target_gripper"]
            )
        self._backend_lock = threading.Lock()  # MuJoCo and the serial bus are not thread-safe (ADR-006)

        self._obs = self._robot.get_observation()
        self._obs_stamp = self.get_clock().now().to_msg()  # /joint_states stamp = read time (03 §5)
        self._effort = []  # normalized load per joint; empty on the real arm until 6.4a
        self._goal = {name: self._obs[f"{name}.pos"] for name in JOINT_NAMES}  # startup pose: no jump to 0
        self._setpoint = dict(self._goal)
        self._bus_errors = 0  # consecutive failed control ticks
        self._bus_lost = False

        self._pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self._cb_joint_command, 10)
        # Timers on the steady clock: the system clock can step back (WSL2: ~2 s every 30 s [MES-37]) and
        # would pause them. Stamps stay on the ROS clock.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self._p["control_rate_hz"], self._control_step, clock=self._steady)
        self.create_timer(
            1.0 / self._p["joint_state_rate_hz"], self._publish_joint_states, clock=self._steady
        )

        self._pub_image = self.create_publisher(Image, "/external_cam/image_raw", 2)
        self._pub_camera_info = self.create_publisher(CameraInfo, "/external_cam/camera_info", 2)
        self._bridge = CvBridge()
        self._stop_camera = threading.Event()
        self._camera = None
        self._camera_thread = None
        if camera_device:  # with the sim arm too: real images, the real arm not needed
            self._camera = self._connect_camera(camera_device)
            if self._camera is not None:
                info = None
                if camera_info_file:
                    info = camera_info_from_yaml(camera_info_file, self._camera.width, self._camera.height)
                else:
                    self.get_logger().warning("No camera_info_file: images without camera_info (6.2b)")
                self._camera_thread = threading.Thread(
                    target=self._real_camera_loop, args=(self._camera, info), name="webcam", daemon=True
                )
        elif self._use_sim:
            self._camera_thread = threading.Thread(target=self._camera_loop, name="sim_camera", daemon=True)
        if self._camera_thread is not None:
            self._camera_thread.start()

        if self._use_sim:  # evaluation tools (ADR-018)
            self.create_service(Trigger, "/sim/randomize", self._srv_randomize)
            if expose_ground_truth:
                self._pub_ground_truth = {
                    key: self.create_publisher(PoseStamped, f"/sim/ground_truth/{key}", 10)
                    for key in GROUND_TRUTH_BODIES
                }
                self.create_timer(1.0 / GROUND_TRUTH_RATE_HZ, self._publish_ground_truth, clock=self._steady)

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
        # robot.bus.disable_torque()  # Comment out this line to control the robot.
        # self.get_logger().info(
        #     "Torque disabled: arm can be moved freely by hand. Remove this part to control the arm."
        # )
        return robot

    def destroy_node(self):
        """Stops the camera thread, brings the arm to its shutdown pose, then disconnects the backend.

        On the real arm, disconnecting disables the torque: the arm falls if it is not at rest (03 §8).
        A lost bus skips the move, and a failed disconnect is logged: the node exits in every case.
        """
        self._stop_camera.set()
        if self._camera_thread is not None:
            self._camera_thread.join()
        if self._camera is not None:
            self._camera.disconnect()
        if self._robot.is_connected:
            if not self._bus_lost:
                self._move_to_shutdown_pose()
            try:
                self._robot.disconnect()
                self.get_logger().info("Robot disconnected.")
            except (OSError, RuntimeError) as error:
                self.get_logger().error(f"Disconnect failed: {error}")
        super().destroy_node()

    def _move_to_shutdown_pose(self):
        """Direct backend calls at the control rate: the executor no longer runs the timers."""
        goal = dict(zip(ARM_JOINT_NAMES, self._shutdown_pose, strict=True))
        velocity = min(self._p["shutdown_velocity"], self._p["max_joint_velocity"])
        try:
            elapsed, error = move_to(
                self._robot,
                self._setpoint,
                goal,
                self._p["control_rate_hz"],
                velocity,
                SHUTDOWN_TIMEOUT,
                self._p["shutdown_tolerance"],
            )
        except (OSError, RuntimeError) as error:
            self.get_logger().error(f"Shutdown pose not reached, bus error: {error}")
            return
        if error <= self._p["shutdown_tolerance"]:
            self.get_logger().info(f"Shutdown pose reached in {elapsed:.1f} s.")
        else:
            self.get_logger().warning(
                f"Shutdown pose not reached after {elapsed:.1f} s (a joint {error:.2f} rad off): "
                "the arm may drop."
            )

    def _cb_joint_command(self, msg: JointState):
        """Stores the goal only (the control timer applies it): clamped, or unchanged if rejected (03 §4)."""
        self._goal, reason = apply_command(self._goal, msg.name, msg.position)
        if reason:
            self.get_logger().warning(f"/joint_command rejected: {reason}", throttle_duration_sec=1.0)

    def _control_step(self):
        """50 Hz: setpoint towards the goal (speed limiter), send it, read the state back.

        A bus error (ConnectionError, motor packet error) is retried at the next ticks; after bus_retries
        retries the driver stops the control and /joint_states, until it is restarted (03 §8).
        """
        if self._bus_lost:
            return
        self._setpoint = step_towards(
            self._setpoint,
            self._goal,
            1.0 / self._p["control_rate_hz"],
            self._p["max_joint_velocity"],
            self._p["max_gripper_velocity"],
        )
        try:
            with self._backend_lock:
                self._robot.send_action({f"{name}.pos": value for name, value in self._setpoint.items()})
                self._obs = self._robot.get_observation()
                if self._use_sim:
                    self._effort = normalized_effort(self._robot.model, self._robot.data, JOINT_NAMES)
        except (OSError, RuntimeError) as error:
            self._bus_errors += 1
            retries = self._p["bus_retries"]
            if self._bus_errors <= retries:
                self.get_logger().warning(f"Bus error {self._bus_errors}/{retries}, retrying: {error}")
            else:
                self._bus_lost = True
                self.get_logger().error(
                    f"Bus lost after {retries} retries ({error}): /joint_states stopped; restart the driver."
                )
            return
        self._bus_errors = 0
        self._obs_stamp = self.get_clock().now().to_msg()

    def _publish_joint_states(self):
        """30 Hz: last observation, arm in rad, gripper converted % -> rad (what RSP expects).

        Silent once the bus is lost: the brain stops on stale /joint_states (ADR-017).
        """
        if self._bus_lost:
            return
        msg = JointState()
        msg.header.stamp = self._obs_stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [
            *(self._obs[f"{name}.pos"] for name in ARM_JOINT_NAMES),
            pct_to_rad(self._obs["gripper.pos"]),
        ]
        msg.effort = self._effort
        self._pub_joint_states.publish(msg)

    def _srv_randomize(self, request, response):
        """Sim only: new cube pose (provided randomize_object), then a new box position (03 §7).

        The answer carries no pose: ground truth stays out of the pipeline (C-06).
        """
        sim = self._robot
        box = sim.model.body("drop_box").id
        with self._backend_lock:
            sim.model.body_pos[box, :2] = PARKED_BOX_XY  # else the cube may settle on a wall
            sim.randomize_object()
            sim.model.body_pos[box, :2] = sample_box_xy(self._box_rng, sim.data.body("pickup_object").xpos)
            mujoco.mj_forward(sim.model, sim.data)
        response.success, response.message = True, "cube and box moved"
        return response

    def _publish_ground_truth(self):
        """10 Hz, expose_ground_truth only: true cube and box poses, for the evaluation bench (ADR-018)."""
        with self._backend_lock:
            poses = {
                key: (self._robot.data.body(body).xpos.copy(), self._robot.data.body(body).xquat.copy())
                for key, body in GROUND_TRUTH_BODIES.items()
            }
        stamp = self.get_clock().now().to_msg()
        for key, (position, (w, x, y, z)) in poses.items():
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = BASE_FRAME  # MuJoCo world = base_link: the base sits at the scene origin
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
            msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
            msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
            self._pub_ground_truth[key].publish(msg)

    def _camera_loop(self):
        """10 Hz, sim only: copy the state under the lock, render the copy outside it (ADR-006).

        Nothing renders the live MjData: the control timer keeps stepping it while a frame renders.
        """
        sim = self._robot
        state = mujoco.MjData(sim.model)
        # Created here: its OpenGL context is bound to this thread.
        renderer = mujoco.Renderer(sim.model, height=sim.camera_height, width=sim.camera_width)
        K = opencv_intrinsics(sim.get_camera_intrinsics())  # OpenCV pixel centers [MES-41]
        info = camera_info(K, sim.camera_width, sim.camera_height)
        next_frame = time.monotonic()
        try:
            while not self._stop_camera.wait(max(0.0, next_frame - time.monotonic())):
                # A late frame shifts the schedule: no catch-up burst.
                next_frame = max(next_frame + 1.0 / self._p["camera_rate_hz"], time.monotonic())
                with self._backend_lock:
                    mujoco.mj_copyData(state, sim.model, sim.data)
                stamp = self.get_clock().now().to_msg()  # image stamp = copy time, like /joint_states
                renderer.update_scene(state, camera=sim.camera_name)
                if not self._publish_frame(renderer.render(), stamp, info):
                    break
        finally:
            renderer.close()

    def _connect_camera(self, device: str) -> OpenCVCamera | None:
        """The webcam in MJPG, the only format with frames through usbip [MES-17]; RGB, as in sim (ADR-007).

        None if it fails: the arm keeps working without images.
        """
        width, height = self._p["camera_width"], self._p["camera_height"]
        camera = OpenCVCamera(
            OpenCVCameraConfig(index_or_path=Path(device), width=width, height=height, fourcc="MJPG")
        )
        try:
            camera.connect()
        except (ConnectionError, RuntimeError) as error:
            if camera.is_connected:
                camera.disconnect()
            self.get_logger().error(f"No camera image: {error}")
            return None
        self.get_logger().info(f"Camera {device}: {width}x{height} at {camera.fps:.0f} fps")
        return camera

    def _real_camera_loop(self, camera, info: CameraInfo | None):
        """10 Hz: the newest frame of the OpenCVCamera reader thread, stamped when it is taken."""
        next_frame = time.monotonic()
        while not self._stop_camera.wait(max(0.0, next_frame - time.monotonic())):
            next_frame = max(next_frame + 1.0 / self._p["camera_rate_hz"], time.monotonic())
            try:
                frame = camera.read_latest()
            except (TimeoutError, RuntimeError) as error:  # no frame for 0.5 s: none published
                self.get_logger().warning(f"Camera: {error}", throttle_duration_sec=5.0)
                continue
            if not self._publish_frame(frame, self.get_clock().now().to_msg(), info):
                break

    def _publish_frame(self, image, stamp, info: CameraInfo | None) -> bool:
        """Publishes an RGB frame and its camera_info with the same header; False once ROS is down."""
        msg = self._bridge.cv2_to_imgmsg(image, encoding="rgb8")
        msg.header.stamp = stamp
        msg.header.frame_id = CAMERA_FRAME
        try:
            self._pub_image.publish(msg)
            if info is not None:
                info.header = msg.header  # same stamp and frame_id: perception pairs them (C-13)
                self._pub_camera_info.publish(info)
        except Exception:
            if rclpy.ok():
                raise
            return False  # SIGINT/SIGTERM shut the context down during the frame: stop quietly
        return True


def camera_info(K, width: int, height: int) -> CameraInfo:
    """CameraInfo of an ideal pinhole camera (sim): K, no distortion, P = [K | 0] (docs/project/03 §6)."""
    info = CameraInfo(width=width, height=height, distortion_model="plumb_bob")
    info.k = [float(v) for v in K.flatten()]
    info.d = [0.0] * 5
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [*info.k[0:3], 0.0, *info.k[3:6], 0.0, *info.k[6:9], 0.0]
    return info


def camera_info_from_yaml(path, width: int, height: int) -> CameraInfo:
    """CameraInfo of a ROS calibration file (camera_calibration format, 6.2b) for a width x height image."""
    calib = yaml.safe_load(Path(path).read_text())
    if (calib["image_width"], calib["image_height"]) != (width, height):
        raise ValueError(
            f"{path} calibrates {calib['image_width']}x{calib['image_height']} images, the camera gives "
            f"{width}x{height}"
        )
    info = CameraInfo(width=width, height=height, distortion_model=calib["distortion_model"])
    info.k = [float(v) for v in calib["camera_matrix"]["data"]]
    info.d = [float(v) for v in calib["distortion_coefficients"]["data"]]
    info.r = [float(v) for v in calib["rectification_matrix"]["data"]]
    info.p = [float(v) for v in calib["projection_matrix"]["data"]]
    return info


def main(args=None):
    # SIGINT and SIGTERM only set an event (#158). rclpy's handlers raise KeyboardInterrupt at any bytecode:
    # one landing while the executor built a Task left it half-built, and its __del__ printed a traceback.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DriverNode()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.destroy_node()  # the context is still valid: the shutdown logs reach /rosout
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
