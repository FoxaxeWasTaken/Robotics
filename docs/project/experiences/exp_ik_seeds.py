"""IK seeds over the whole joint space: how many reachable targets does ik_pos miss? Feeds MES-36.

2000 random arm configurations within the URDF limits; target = FK; ik_pos with the seeds of 04 §3.1 only
(pan = -azimuth, three arm shapes), then with the fallback grid of so101_brain/kinematics.py.
In the control container, workspace built: python3 docs/project/experiences/exp_ik_seeds.py
"""
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory

from so101_brain import kinematics

kin = kinematics.Kinematics(get_package_share_directory("so101_description") + "/urdf/so101.urdf")
low = np.array([link.bounds[0] for link in kin._chain.links[1:6]])
high = np.array([link.bounds[1] for link in kin._chain.links[1:6]])
configurations = np.random.default_rng(1).uniform(low, high, size=(2000, 5))
fallback = kinematics.FALLBACK_TAILS

for label, tails in (("04 §3.1 seeds only", ()), ("with the fallback grid", fallback)):
    kinematics.FALLBACK_TAILS = tails
    misses, behind, times = 0, 0, []
    for q in configurations:
        p = kin.fk(q)[:3, 3]
        t0 = time.perf_counter()
        solution = kin.ik_pos(p)
        times.append(time.perf_counter() - t0)
        if solution is None:
            misses += 1
            behind += abs(kin.arm_azimuth(p)) > high[0]
    print(
        f"{label}: {misses} misses / 2000 ({behind} with |azimuth| past the pan limit); "
        f"time median {1000 * np.median(times):.0f} ms, p99 {1000 * np.percentile(times, 99):.0f} ms"
    )
