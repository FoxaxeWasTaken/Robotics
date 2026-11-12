"""E4: reachable approach tilts per column with the roll locked: ik_point(p, tilted_down(t, az), roll) vs ik_pos."""
import numpy as np, time
from so101_brain.kinematics import Kinematics, PAN_AXIS_X
from so101_brain.grasp_planner import tilted_down
URDF = "/repo/ros2_ws/src/so101_description/urdf/so101.urdf"
k = Kinematics(URDF)
ROLL = np.pi / 2 + 0.0487
Rs = [0.14 + 0.037 * i for i in range(7)]
n = np.array([0, 1.0, 0])
TILTS = list(range(0, 95, 5))
for z in (0.195, 0.205, 0.215, 0.235):
    print(f"z={z}")
    for r in Rs:
        p = np.array([PAN_AXIS_X + r, -0.008, z])
        ok = []; ts = []; tail = {}
        for t in TILTS:
            t0 = time.perf_counter()
            q = k.ik_point(p, tilted_down(t, 0.0), roll=ROLL)
            ts.append(time.perf_counter() - t0)
            if q is not None:
                T = k.fk(q)
                ok.append(t); tail[t] = (q, np.degrees(np.arccos(min(1, abs(T[:3, 0] @ n)))), np.linalg.norm(T[:3,3]-p)*1e3)
        best = ok[0] if ok else None
        print(f"  r={r:.3f}: tilts ok {ok}; t median {np.median(ts)*1e3:.0f} ms max {max(ts)*1e3:.0f} ms" + (f"; lowest {best}: q={np.round(tail[best][0],3).tolist()} jaw {tail[best][1]:.2f} fk {tail[best][2]:.2f} mm" if ok else ""))
# unreachable target: worst IK time
for p in ([0.7, 0, 0.2], [0.05, 0.0, 0.05]):
    t0 = time.perf_counter(); q = k.ik_pos(np.array(p), roll=ROLL); print("unreachable", p, q, f"{(time.perf_counter()-t0)*1e3:.0f} ms")
