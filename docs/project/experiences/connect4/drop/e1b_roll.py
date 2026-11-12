"""E1b: is the 2.8 deg residual a roll zero? Scan the locked roll offset, then a grid plane offset from the pan axis."""
import numpy as np
from so101_brain.kinematics import Kinematics, PAN_AXIS_X, SEED_TAILS
URDF = "/repo/ros2_ws/src/so101_description/urdf/so101.urdf"
k = Kinematics(URDF)
R = [0.14 + 0.037 * i for i in range(7)]
def jaw(T, n):
    return np.degrees(np.arccos(min(1.0, abs(T[:3, 0] @ n))))
def seed_for(r):
    # a seed that keeps the arm in the elbow-up, tool-down family
    return np.array([0, -0.5, 0.0, 1.2, 0.0])
n = np.array([0, 1.0, 0])
for z in (0.20,):
  for sign in (+1, -1):
    for d in (0, -1.5, -2.79, -4, +1.5, +2.79):
        for off in (0.0, +0.008, -0.008):
            errs, pans = [], []
            for r in R:
                p = np.array([PAN_AXIS_X + r, off, z])
                roll = sign * np.pi / 2 + np.radians(d)
                q = k.ik_pos(p, q_seed=seed_for(r), roll=roll)
                errs.append(np.nan if q is None else jaw(k.fk(q), n)); pans.append(np.nan if q is None else q[0])
            print(f"z={z} roll={sign*90:+d}{d:+.2f} deg, plane offset y={off*1e3:+.0f} mm: jaw {np.round(errs,2).tolist()} pan {np.round(pans,4).tolist()}")
