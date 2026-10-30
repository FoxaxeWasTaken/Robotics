"""Real arm: each joint's reading at the URDF zero (MES-55, #196), the joint_zero_offsets of driver_real.yaml.

LeRobot's zero is the middle of each recorded range, not the URDF's (07 §4, MES-30). Read-only: the stack
stopped (one program on the bus), the arm held by hand at the URDF zero (stretched forward, horizontal), torque
off; writes nothing to the motors. From the repository root, with the arm's calibration folder:

    docker run --rm -it --network none --device /dev/ttyACM0 -v <calibration folder>:/cal \
      -v "$PWD/docs/project/experiences:/e:ro" tek5/control:latest python3 /e/exp_zero_offsets.py
"""

from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

robot = SO101Follower(
    SO101FollowerConfig(port="/dev/ttyACM0", id="arm", calibration_dir=Path("/cal"), use_radians=True)
)
robot.bus.connect()  # the bus only: no configure(), so the torque stays as it is (off)
try:
    read = robot.bus.sync_read("Present_Position")
finally:
    robot.bus.disconnect(disable_torque=False)  # writes nothing
print({name: round(read[name], 3) for name in ARM})
print(f"joint_zero_offsets: [{', '.join(f'{read[name]:.3f}' for name in ARM)}]")
