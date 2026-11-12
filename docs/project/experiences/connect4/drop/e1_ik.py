"""E1: the real Kinematics on the 7 columns: ik_pos(p, roll=+-pi/2), seeds, FK and jaw errors, rejections, time."""
import time, sys
import numpy as np
from so101_brain.kinematics import Kinematics, PAN_AXIS_X
URDF = "/repo/ros2_ws/src/so101_description/urdf/so101.urdf"
rej = {"self": 0}
class K(Kinematics):
    def self_collides(self, q):
        r = super().self_collides(q)
        rej["self"] += int(r)
        return r
k = K(URDF)
R = [0.14 + 0.037 * i for i in range(7)]
HANDOVER_SEEDS = {"none": None, "bent": np.array([0.0, -0.3, 0.3, 1.2, 0.0])}
def jaw(T, n, u):
    x = T[:3, 0]
    err = np.degrees(np.arccos(min(1.0, abs(x @ n))))
    s = np.sign(x @ n)
    yaw = np.degrees(np.arctan2(s * x @ u, s * x @ n))    # about the vertical: along the column line
    rollv = np.degrees(np.arctan2(s * x[2], s * x @ n))   # about the radial axis: top/bottom of the disc
    return err, yaw, rollv
def tilt(T):  # approach z axis tilt from straight down, deg
    return np.degrees(np.arccos(np.clip(-T[2, 2], -1, 1)))
for az_deg in (0.0, 45.0, -45.0, 90.0):
    az = np.radians(az_deg)
    u = np.array([np.cos(az), np.sin(az), 0.0])
    n = np.array([-np.sin(az), np.cos(az), 0.0])
    print(f"\n=== grid azimuth {az_deg:+.0f} deg (plane through the pan axis)")
    for z in (0.20, 0.215, 0.23, 0.25):
        for sign in (+1, -1):
            roll = sign * np.pi / 2
            rows, ts = [], []
            for r in R:
                p = np.array([PAN_AXIS_X, 0, z]) + r * u
                rej["self"] = 0
                t0 = time.perf_counter()
                q = k.ik_pos(p, roll=roll)
                dt = time.perf_counter() - t0; ts.append(dt)
                if q is None:
                    rows.append(f"r={r:.3f} NONE rej={rej['self']}")
                    continue
                T = k.fk(q)
                e, yaw, rv = jaw(T, n, u)
                rows.append(f"r={r:.3f} q={np.round(q,3).tolist()} fk={np.linalg.norm(T[:3,3]-p)*1e3:.2f}mm jaw={e:.2f} yaw={yaw:+.2f} rollv={rv:+.2f} tilt={tilt(T):.1f} rej={rej['self']} {dt*1e3:.0f}ms")
            print(f"-- z={z} roll={sign*90:+.0f}: time median {np.median(ts)*1e3:.0f} ms max {max(ts)*1e3:.0f} ms")
            for row in rows:
                print("   ", row)
