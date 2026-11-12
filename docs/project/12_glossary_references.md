# 12 — Glossary and references

## 1. Glossary

| Term | Definition (in this project) |
| --- | --- |
| **ADR** | *Architecture Decision Record*: a short record of a decision (context, options, decision, consequences, evidence) |
| **DoF** | Degree of freedom. SO-101: 5 DoF + gripper |
| **FK / IK** | Forward kinematics (angles → TCP pose) / inverse kinematics (desired pose → angles) |
| **TCP** | *Tool Center Point*: the gripper's working point = `gripper_frame_link` |
| **Approach axis** | Direction along which the gripper reaches the object (+z of the TCP) |
| **DLS** | *Damped Least Squares*: iterative IK robust near singularities |
| **Multi-start** | Running IK from several seeds and keeping the best valid solution |
| **Jacobian** | Matrix linking joint velocities to TCP velocities |
| **URDF / MJCF** | Robot description formats: ROS (URDF) and MuJoCo (MJCF) |
| **REP-103 / REP-105** | ROS conventions: SI units and frames (103), standard frame names (105) |
| **TF** | Tree of transforms between frames (`/tf`, `/tf_static`) |
| **RSP** | `robot_state_publisher`: computes the robot TFs from `/joint_states` and the URDF |
| **QoS** | DDS quality of service: reliability (Reliable / Best effort), durability (Volatile / Transient local), queue depth |
| **DDS** | Decentralized network middleware under ROS 2 |
| **`ROS_DOMAIN_ID`** | Isolates groups of nodes on the same network |
| **Callback group / Executor** | rclpy mechanisms to run callbacks in parallel or mutually exclusively |
| **colcon / ament** | ROS 2 build tool / package systems (`ament_python`, `ament_cmake`) |
| **HSV** | Hue-Saturation-Value color space; OpenCV uses H ∈ [0, 180] |
| **Intrinsics (K)** | Internal camera parameters: focal lengths fx, fy, principal point cx, cy (+ distortion D) |
| **Extrinsics** | Camera pose in the robot frame (rotation + translation) |
| **PnP** | *Perspective-n-Point*: recovering the camera pose from 3D ↔ 2D correspondences (`cv2.solvePnP`) |
| **ArUco / ChArUco / AprilTag** | Identifiable square fiducial markers; ChArUco = checkerboard + ArUco, for calibration |
| **Reprojection error** | Pixel distance between detected points and reprojected 3D points: calibration quality indicator |
| **Optical frame** | Camera convention: z forward, x right, y down (different from MuJoCo) |
| **Minimum jerk** | Quintic trajectory profile, zero velocity and acceleration at both ends |
| **FSM** | *Finite State Machine* |
| **Ground truth** | Exact sim values (`get_object_position`…): tests and the evaluation bench only (C-06) |
| **Spike** | Throwaway experiment answering a technical question by measurement |
| **Vertical slice** | First end-to-end version of the whole chain, even simplistic |
| **rosbag2** | Recording and replay of ROS 2 topics |
| **OSMesa** | Software (CPU) OpenGL rendering used by MuJoCo in the container |
| **usbip / usbipd-win** | Sharing a Windows USB device with WSL2 |
| **UVC / MJPG / YUYV** | Webcam USB class / compressed JPEG format / raw uncompressed format |
| **Sim2real** | Gap between simulation and reality, and techniques to reduce it |
| **Mirror** | Read-only copy of a repository kept in sync automatically (here: private repo → school repo) |
| **Bus factor** | Number of people whose absence blocks the project |
| **Phase 1 / Phase 2** | The brief's two parts: the pick & place project / your own application reusing it (both graded at the final defense) |
| **Stage** | A work package of the roadmap (`09`, stages 0–8) and an epic on the GitHub Project; not to be confused with the brief's phases |
| **Edge-on grid** | Phase 2: Connect Four grid whose plane contains the `shoulder_pan` axis, so the arm works in the plane of the grid (ADR-035) |
| **Jaw axis** | x axis of `gripper_frame_link`, the direction in which the jaws close; normal to the grid when dropping a token |
| **Drop pose** | TCP pose that drops a token into a column: jaws normal to the grid, token just above the depth guide (`phase2/P2-02` §2) |
| **Depth guide** | Two sloped strips along the top of the grid, a V across the slots: absorbs the arm's error in the tight direction (`phase2/P2-02` §1) |
| **Handover** | The human puts the robot's token between its open jaws, then presses "token given"; the gripper closes, the arm stays still (`phase2/P2-02` §5) |
| **Riser** | Rigid block under the arm (about 8 cm) so that a store-bought grid is within reach [MES-32] |
| **Wrist camera (eye-in-hand)** | Camera on the gripper; sees the token and the slot in one image, so their offset does not depend on calibration (`phase2/P2-02` §3.1) |
| **Negamax, alpha-beta** | Game-tree search of the Connect Four engine; alpha-beta skips the branches that cannot change the result |
| **Task layer** | Phase-2 node that turns a game move into a `/brain/drop` call (ADR-037) |
| **P2-xx, P2-R-xx** | Phase-2 targets set by the team (`phase2/P2-01` §4) and phase-2 risks (`phase2/P2-03` §2) |

## 2. References

### ROS 2

- ROS 2 Jazzy documentation: <https://docs.ros.org/en/jazzy/>
- REP-103 (units and conventions): <https://www.ros.org/reps/rep-0103.html>
- REP-105 (frames): <https://www.ros.org/reps/rep-0105.html>
- QoS: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
- Callback groups and executors: <https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html>

### Simulation, kinematics

- MuJoCo: <https://mujoco.readthedocs.io/>
- Pinocchio: <https://github.com/stack-of-tasks/pinocchio>
- ikpy: <https://github.com/Phylliade/ikpy>
- Pink (QP differential IK on Pinocchio): <https://github.com/stephane-caron/pink>
- mink (differential IK on MuJoCo): <https://github.com/kevinzakka/mink>
- K. Lynch, F. Park, *Modern Robotics* (free book and course): <https://hades.mech.northwestern.edu/index.php/Modern_Robotics>

### Vision

- OpenCV 4.x (calib3d and objdetect/aruco modules): <https://docs.opencv.org/4.x/>
- R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision* (camera geometry reference)
- P. Corke, *Robotics, Vision and Control* (practical robotics and vision)

### Hardware and ecosystem

- LeRobot SO-101: <https://huggingface.co/docs/lerobot/so101>
- usbipd-win: <https://github.com/dorssel/usbipd-win>

### Process and tooling

- py_trees (behavior trees, phase-2 option): <https://github.com/splintered-reality/py_trees>
- ADR format (Michael Nygard): <https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions>
- Conventional Commits: <https://www.conventionalcommits.org/>
- Keep a Changelog: <https://keepachangelog.com/en/1.1.0/>
- pre-commit: <https://pre-commit.com/> · ruff: <https://docs.astral.sh/ruff/>
