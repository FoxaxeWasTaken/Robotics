"""Fault scenarios 1 and 2 of step 4.4a (#54), run against the sim stack: prints each /brain_state change.

Scenario 1, "approach": /sim/randomize, /brain/start, then /sim/randomize again on entering APPROACH: the cube
leaves before the grasp; expected GRASP -> RETRY (empty gripper), a new lock, then DONE.
Scenario 2, "watch": /sim/randomize, /brain/start, then only the states; the host stops the perception
container when TRANSPORT shows; expected: the cycle completes (C-33), VERIFY ends in DONE with a warning.
The /brain_state lines are flushed one by one, so the host can react to them. No ground truth (C-06).

With the stack up (tools/so101 up), from the repository root:
    C="docker compose -f docker/docker-compose.yml"
    $C exec -T control bash -lc "source /ros2_ws/install/setup.bash && python3 - approach" \
        < docs/project/experiences/exp_fault_scenarios.py
    $C exec -T control bash -lc "source /ros2_ws/install/setup.bash && python3 - watch" \
        < docs/project/experiences/exp_fault_scenarios.py | while IFS= read -r line; do
        echo "$line"; [[ $line == *"  TRANSPORT" ]] && $C stop -t 5 perception &
    done; wait; $C start perception
The brain's reasons: $C logs --since <start time> control | grep so101_brain
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

CYCLE_TIMEOUT_S = 120.0
END = ("DONE", "STOPPED")

mode = sys.argv[1]
assert mode in ("approach", "watch"), mode
rclpy.init()
node = rclpy.create_node("exp_fault_scenarios")
latched = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
states, poses = [], []
node.create_subscription(String, "/brain_state", lambda m: states.append(m.data), latched)
node.create_subscription(PoseStamped, "/end_effector_pose", lambda m: poses.append(m), 10)
randomize, start = (node.create_client(Trigger, s) for s in ("/sim/randomize", "/brain/start"))


def call(client):
    assert client.wait_for_service(timeout_sec=10.0), client.srv_name
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result().message


deadline = time.monotonic() + 30.0
while not poses and time.monotonic() < deadline:  # the brain has /joint_states (AGENTS.md, #132)
    rclpy.spin_once(node, timeout_sec=0.1)
print("randomize:", call(randomize), flush=True)
time.sleep(2.0)  # the cube settles
t0 = time.monotonic()
print("start:", call(start), flush=True)
last, injected = None, False
while time.monotonic() - t0 < CYCLE_TIMEOUT_S:
    rclpy.spin_once(node, timeout_sec=0.05)
    while states:
        state = states.pop(0)
        if state == last:
            continue
        last = state
        print(f"{time.monotonic() - t0:6.2f} s  {state}", flush=True)
        if mode == "approach" and state == "APPROACH" and not injected:
            injected = True
            print(f"{time.monotonic() - t0:6.2f} s  >>> /sim/randomize: {call(randomize)}", flush=True)
    if last in END or (last or "").startswith("ERROR"):
        break
node.destroy_node()
rclpy.shutdown()
