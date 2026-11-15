"""Real arm: the gripper's Present_Load empty and on the cube (MES-56, #69), the effort the driver plans (03 §5).

Run by a human, the stack stopped, the arm resting: only the gripper moves. Closing at 0 % on the 5 cm cube puts
the motor in overload protection (MES-56): a power cycle clears it. From the repository root:

    docker run --rm -it --network none --device /dev/ttyACM0 -v <calibration folder>:/cal \
      -v "$PWD/docs/project/experiences:/e:ro" tek5/control:latest python3 /e/exp_gripper_load.py
"""

import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

robot = SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="arm", calibration_dir=Path("/cal")))
robot.connect(calibrate=False)  # torque on: the arm holds where it rests (the motors keep their pose)


def measure(label, percent, wait=1.5, samples=20):
    robot.send_action({"gripper.pos": percent})  # the gripper only, in % (0 closed, 100 open)
    time.sleep(wait)
    loads = []
    for _ in range(samples):
        loads.append(robot.bus.read("Present_Load", "gripper"))
        time.sleep(0.05)
    mean = sum(loads) / len(loads)
    print(f"{label:30} {percent:5.1f} %   Present_Load {min(loads)}..{max(loads)}   /1000 = {mean / 1000:+.3f}")


try:
    measure("open, empty", 100.0)
    measure("closed, empty", 0.0)
    measure("open", 100.0)
    input("Hold the cube between the jaws, then ENTER to close...")
    measure("closed on the cube", 0.0, wait=2.0)
    input("Let go of the cube (the gripper holds it), then ENTER...")
    measure("closed on the cube, let go", 0.0, wait=1.0)
    input("Take the cube back, then ENTER to open...")
    measure("open", 100.0)
finally:
    robot.disconnect()  # torque off: keep the arm resting on its support
