"""The Python side of tools/so101-arm (#199), in a throwaway control container with only the arm's port:

    python3 so101_arm.py calibrate <port> <id>
    python3 so101_arm.py zero-offsets <port> <id> <readings>

The calibration file is /calibration/<id>.json. Both open the bus alone: SO101Follower.connect() would run
configure(), which turns the torque on. Only LeRobot's calibrate() writes to the motors.
"""

import statistics
import sys
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
HALF_TURN = 2047  # the reading calibrate() gives the pose held at "middle": LeRobot's zero (07 §4)
PROMPTS = """At the prompts (07 §3.1):
  1. "type 'c'": c then ENTER to recalibrate (ENTER alone writes the current file to the motors);
  2. "middle of its range": hold the arm at the URDF zero (stretched forward, horizontal), then ENTER;
  3. each joint but wrist_roll to both its stops, one after the other, then ENTER.
The torque is off from step 2: hold the arm."""


def calibrate(robot):
    print(PROMPTS)
    robot.calibrate()
    for motor, c in robot.calibration.items():
        middle = (c.range_min + c.range_max) / 2
        offset = middle - HALF_TURN
        print(f"{motor:14} range {c.range_min:4}-{c.range_max:4}  middle {middle:.1f} ({offset:+.1f})")


def zero_offsets(robot, readings):
    reads = []
    for i in range(readings):
        if i:
            time.sleep(0.5)
        reads.append(robot.bus.sync_read("Present_Position"))
    means = {name: statistics.fmean(r[name] for r in reads) for name in ARM}
    for name in ARM:
        spread = max(r[name] for r in reads) - min(r[name] for r in reads)
        print(f"{name:14} {means[name]:+.3f} rad  spread {spread:.3f} over {readings} reading(s)")
    print(f"joint_zero_offsets: [{', '.join(f'{means[name]:.3f}' for name in ARM)}]")


def main(command, port, name, readings="1"):
    config = SO101FollowerConfig(port=port, id=name, calibration_dir=Path("/calibration"), use_radians=True)
    robot = SO101Follower(config)
    robot.bus.connect()  # the bus only: the torque stays as it is
    try:
        if command == "calibrate":
            calibrate(robot)
        else:
            zero_offsets(robot, int(readings))
    finally:
        robot.bus.disconnect(disable_torque=False)  # writes nothing


if __name__ == "__main__":
    main(*sys.argv[1:])
