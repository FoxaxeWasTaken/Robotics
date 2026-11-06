"""The Python side of tools/so101-arm (#199), in a throwaway control container with only the arm's port:

    python3 so101_arm.py calibrate <port> <id>
    python3 so101_arm.py zero-offsets <port> <id> <readings>
    python3 so101_arm.py touch <port> <id> <board corner>

The calibration file is /calibration/<id>.json. All open the bus alone: SO101Follower.connect() would run
configure(), which turns the torque on. Only LeRobot's calibrate() writes to the motors. touch needs the
sources at /src: the URDF, the kinematics and driver_real.yaml.
"""

import statistics
import sys
import time
from pathlib import Path

import yaml
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
HALF_TURN = 2047  # the reading calibrate() gives the pose held at "middle": LeRobot's zero (07 §4)
MIN_RANGE = (
    500  # steps: a joint that recorded less was not moved at the last prompt (2026-11-06: the gripper, 1)
)
PROMPTS = """At the prompts (07 §3.1):
  1. "type 'c'": c then ENTER to recalibrate (ENTER alone writes the current file to the motors);
  2. "middle of its range": hold the arm at the URDF zero (stretched forward, horizontal), then ENTER;
  3. each joint but wrist_roll to both its stops, one after the other, the GRIPPER too, then ENTER.
The torque is off from step 2: hold the arm."""


def calibrate(robot):
    print(PROMPTS)
    robot.calibrate()
    for motor, c in robot.calibration.items():
        middle = (c.range_min + c.range_max) / 2
        offset = middle - HALF_TURN
        print(f"{motor:14} range {c.range_min:4}-{c.range_max:4}  middle {middle:.1f} ({offset:+.1f})")
    short = [motor for motor, c in robot.calibration.items() if c.range_max - c.range_min < MIN_RANGE]
    if short:
        sys.exit(f"Not moved to its stops: {', '.join(short)}. Calibrate again, or that joint will not move.")


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


def tcp(readings, offsets):
    """The TCP in base_link (m) of the arm joints read, in the driver's URDF angles (07 §4)."""
    sys.path[:0] = ["/src/so101_brain", "/src/so101_common"]
    from so101_brain.kinematics import Kinematics

    q = [reading - offset for reading, offset in zip(readings, offsets, strict=True)]
    return Kinematics("/src/so101_description/urdf/so101.urdf").fk(q)[:3, 3]


def touch(robot, corner):
    read = robot.bus.sync_read("Present_Position")
    config = yaml.safe_load(Path("/src/so101_bringup/config/driver_real.yaml").read_text())
    offsets = config["so101_driver"]["ros__parameters"]["joint_zero_offsets"]
    x, y, z = tcp([read[name] for name in ARM], offsets)
    print(f"--touch {corner} {x:.5f} {y:.5f} {z:.5f}")


def main(command, port, name, argument="1"):
    config = SO101FollowerConfig(port=port, id=name, calibration_dir=Path("/calibration"), use_radians=True)
    robot = SO101Follower(config)
    robot.bus.connect()  # the bus only: the torque stays as it is
    try:
        if command == "calibrate":
            calibrate(robot)
        elif command == "touch":
            touch(robot, argument)
        else:
            zero_offsets(robot, int(argument))
    finally:
        robot.bus.disconnect(disable_torque=False)  # writes nothing


if __name__ == "__main__":
    main(*sys.argv[1:])
