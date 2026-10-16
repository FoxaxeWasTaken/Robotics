"""Real arm: does LeRobot's joint zero match the URDF zero? Feeds MES-30.

LeRobot (RADIANS mode) reports angle = (raw - (range_min + range_max) / 2) * 2*pi / 4095: the zero is the
MIDDLE OF THE RECORDED RANGE. The pose held during calibration reads raw = 2047 (half-turn homing).
If the URDF zero is that held pose, every joint is offset by (mid - 2047) ticks.
Usage: edit CALIB (copy of the arm's calibration JSON) and REST (/joint_states at rest), then run.
"""
import warnings, numpy as np
warnings.filterwarnings("ignore")
from ikpy.chain import Chain

# Team arm, 2026-10-09 (calibration read from the motors; rest pose measured with `ros2 topic echo /joint_states`).
CALIB = {"shoulder_pan": (759, 3381), "shoulder_lift": (808, 2766), "elbow_flex": (833, 3065),
         "wrist_flex": (839, 3183), "wrist_roll": (0, 4095)}
REST = np.array([0.155, -1.487, 1.700, 1.309, 0.007])  # rad, folded rest pose

chain = Chain.from_urdf_file("/ros2_ws/src/so101_description/urdf/so101.urdf", base_elements=["base_link"],
                             active_links_mask=[False] + [True] * 5 + [False])
mid = np.array([(lo + hi) / 2 for lo, hi in CALIB.values()])
offset = (mid - 2047) * 2 * np.pi / 4095
print("held calibration pose vs reported zero, per joint (deg):", dict(zip(CALIB, np.degrees(offset).round(1))))
lo, hi = np.array([l.bounds for l in chain.links[1:6]]).T
for name, q in [("as reported by LeRobot", REST), ("if URDF zero = held calibration pose", REST + offset)]:
    p = chain.forward_kinematics(np.r_[0, q, 0])[:3, 3]
    outside = [n for n, v, a, b in zip(CALIB, q, lo, hi) if not a <= v <= b]
    print(f"rest pose, {name:38s}: TCP {p.round(3)} m (z = {p[2] * 100:+.1f} cm); outside URDF limits: {outside or 'none'}")
print("Check on the real arm: measure the gripper tip at rest (distance in front of the base, height) and compare.")
