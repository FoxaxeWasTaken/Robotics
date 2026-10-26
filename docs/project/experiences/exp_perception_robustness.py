"""MES-46: what perception publishes with the moved sim camera (#47, step 3.5): error and publication rate over
/sim/randomize scenes with the arm in PARK, and outliers while the arm crosses the camera's view.

Run next to a stack with the ground truth and perception, on its own ROS domain (the ground truth only scores
perception here, C-06):
    ROS_DOMAIN_ID=92 ros2 launch so101_bringup sim.launch.py expose_ground_truth:=true perception:=true &
    ROS_DOMAIN_ID=92 python3 exp_perception_robustness.py [scenes]

Each scene: /sim/randomize, then, once the cube has settled (a cube dropped against the arm's base may slide),
2 s of /object_position, each against the ground truth of its moment.
Every 10th scene: /go_to_target 10 cm from the cube towards the camera, then back to PARK: the arm passes
between them and hides the cube; whatever perception publishes meanwhile is scored too, and the images without
a publication show it was hidden. Only messages stamped after the scene change
count. An outlier: a published cube center more than 2 cm from the ground truth.
"""

import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from so101_interfaces.srv import GoToTarget

SCENES = int(sys.argv[1]) if len(sys.argv) > 1 else 200
PARK = {"shoulder_pan": 1.9, "shoulder_lift": -1.7, "elbow_flex": 1.5, "wrist_flex": 0.7, "wrist_roll": 0.0}
OUTLIER_M = 0.02
CAMERA = np.array(yaml.safe_load(open("/ros2_ws/src/so101_bringup/config/sim_scene.yaml"))["camera"]["pos"])

rclpy.init()
node = rclpy.create_node("exp_perception_robustness")
published, truth, pose = [], {}, {}


def xyz(msg):
    return np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


def tilt(msg):
    """Angle between the cube's z axis and the vertical, degrees (0: resting flat on a face)."""
    q = msg.pose.orientation
    return np.degrees(np.arccos(np.clip(1 - 2 * (q.x**2 + q.y**2), -1.0, 1.0)))


node.create_subscription(PoseStamped, "/object_position", lambda m: published.append((Time.from_msg(m.header.stamp), xyz(m))), 50)
node.create_subscription(PoseStamped, "/sim/ground_truth/object", lambda m: truth.update(cube=xyz(m), tilt=tilt(m)), 10)
node.create_subscription(PoseStamped, "/sim/ground_truth/drop_box", lambda m: truth.update(box=xyz(m)), 10)
node.create_subscription(PoseStamped, "/end_effector_pose", lambda m: pose.update(tcp=xyz(m)), 10)
node.create_subscription(JointState, "/joint_states", lambda m: pose.update(joints=dict(zip(m.name, m.position))), 10)
command = node.create_publisher(JointState, "/joint_command", 10)
randomize, go = node.create_client(Trigger, "/sim/randomize"), node.create_client(GoToTarget, "/go_to_target")


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(client, request):
    future = client.call_async(request)
    end = time.monotonic() + 30
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.result()


def park():
    """Until /joint_states shows PARK: a /go_to_target trajectory may still be streaming over the command."""
    end = time.monotonic() + 20
    while time.monotonic() < end:
        command.publish(JointState(name=list(PARK), position=list(PARK.values())))  # partial command
        spin_for(0.5)
        if all(abs(pose["joints"][n] - q) < 0.02 for n, q in PARK.items()):
            return
    raise RuntimeError("PARK not reached")


for client in (randomize, go):
    assert client.wait_for_service(timeout_sec=60), client.srv_name
end = time.monotonic() + 60
while not ("cube" in truth and "tcp" in pose) and time.monotonic() < end:  # the brain sees /joint_states
    rclpy.spin_once(node, timeout_sec=0.1)
park()

static_errors, seen, motion_errors, motion_scenes, moved, refused, frames, slid = [], 0, [], 0, 0, 0, 0, 0
for k in range(SCENES):
    assert call(randomize, Trigger.Request()).success
    truth.clear()
    settled, end, last = 0, time.monotonic() + 5.0, None
    while settled < 5 and time.monotonic() < end:  # 5 ground-truth messages (0.5 s) within 0.5 mm
        spin_for(0.1)
        now = truth.get("cube")
        settled = settled + 1 if last is not None and now is not None and np.linalg.norm(now - last) < 5e-4 else 0
        last = now
    slid += settled < 5
    since = node.get_clock().now()
    spin_for(2.0)
    cube = truth["cube"]
    errors = [np.linalg.norm(p - cube) for t, p in published if t > since]
    seen += bool(errors)
    static_errors += errors
    if errors and max(errors) > OUTLIER_M:
        print(f"  scene {k}: {sum(e > OUTLIER_M for e in errors)}/{len(errors)} poses off, max {1e3 * max(errors):.1f} mm; "
              f"cube {np.round(cube, 3).tolist()} tilted {truth['tilt']:.1f} deg, published mean "
              f"{np.round(np.mean([p for t, p in published if t > since], axis=0), 3).tolist()}, box "
              f"{np.round(truth['box'], 3).tolist()}", flush=True)
    if k % 10 == 9:
        since = node.get_clock().now()
        target = PoseStamped()
        target.header.frame_id = "base_link"
        between = cube + 0.10 * (CAMERA - cube) / np.linalg.norm(CAMERA - cube)
        target.pose.position.x, target.pose.position.y, target.pose.position.z = between
        if not call(go, GoToTarget.Request(target_pose=target)).success:  # out of reach
            refused += 1
            park()
            published.clear()
            continue
        spin_for(4.0)
        park()
        if np.linalg.norm(truth["cube"] - cube) > 0.002:  # the arm touched the cube: not a perception error
            moved += 1
        else:
            motion_scenes += 1
            frames += 82  # 10 Hz over the 8.2 s of motion
            motion_errors += [np.linalg.norm(p - cube) for t, p in published if t > since]
    published.clear()

e = np.array(static_errors)
print(f"arm in PARK, {SCENES} scenes ({slid} still moving after 5 s): cube published in {seen} ({100 * seen / SCENES:.1f} %), {len(e)} poses; error "
      f"mean {1e3 * e.mean():.1f} mm, p95 {1e3 * np.percentile(e, 95):.1f} mm, max {1e3 * e.max():.1f} mm; "
      f"outliers > {100 * OUTLIER_M:.0f} cm: {np.sum(e > OUTLIER_M)}")
m = np.array(motion_errors) if motion_errors else np.zeros(0)
print(f"arm between the camera and the cube, {motion_scenes} scenes ({moved} skipped, cube touched; {refused} out of "
      f"reach): {len(m)} poses published in ~{frames} images"
      + (f", max error {1e3 * m.max():.1f} mm, outliers > {100 * OUTLIER_M:.0f} cm: {np.sum(m > OUTLIER_M)}" if len(m) else ""))
