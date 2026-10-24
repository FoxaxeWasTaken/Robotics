"""evaluate - the evaluation bench in sim (docs/project/08 §3).

    ros2 run so101_bringup evaluate --episodes N --out <csv> [--bags <dir>] [--seed S] [--timeout 60]
                                    [--min-success M]

Each episode: /sim/randomize (cube and box), /brain/start from IDLE, then DONE, ERROR or the bench timeout
(then /brain/stop). Scored with the driver's ground truth (expose_ground_truth, for this bench only, C-06):
success = DONE with the cube within the box footprint, z < 6 cm. One CSV row per episode, a rosbag per
failed episode with --bags, and the summary at the end. --seed checks the seed the driver runs. With
--min-success, fewer than M successes exit 1, after the CSV and the summary (the PR's E2E job, #156).
The checks before the first episode (driver, ground truth, brain pose) wait up to 60 s each, for a fresh
process on a CI runner; the calls during the episodes wait 10 s.
"""

import argparse
import csv
import shutil
import signal
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

FOOTPRINT_HALF_M = 0.05  # half the box's inner side (08 §3)
MAX_CUBE_Z_M = 0.06  # higher: on a wall, not in the box
STARTUP_S = 60.0  # s, each check before episode 1: a fresh process may find the stack late in CI (#141)
RESEND_S = 5.0  # s: the driver's parameters asked again without a reply (none in 60 s once in CI, #141)
FIELDS = [
    "episode", "seed", "cube_x", "cube_y", "cube_z", "box_x", "box_y", "box_z",
    "final_state", "duration_s", "success", "cause",
]  # fmt: skip
BAG_TOPICS = [
    "/joint_states", "/joint_command", "/object_position", "/drop_box_position", "/brain_state",
    "/sim/ground_truth/object", "/sim/ground_truth/drop_box",
]  # fmt: skip


def outcome(final_state: str, cube, box) -> tuple[bool, str]:
    """(success, failure cause) of an episode; the cause is empty on success."""
    if final_state == "TIMEOUT":
        return False, "bench_timeout"
    if final_state != "DONE":
        return False, final_state.removeprefix("ERROR:")
    inside = bool(
        np.all(np.abs(np.subtract(cube[:2], box[:2])) < FOOTPRINT_HALF_M) and cube[2] < MAX_CUBE_Z_M
    )
    return inside, "" if inside else "not_in_box"


def summary(rows) -> str:
    """Success rate, p50/p95 duration of the successful cycles (a timeout is not a duration), causes."""
    done = [r["duration_s"] for r in rows if r["success"]]
    rate = f"success {len(done)}/{len(rows)} ({100 * len(done) / len(rows):.0f} %)"
    duration = (
        f"duration p50 {np.percentile(done, 50):.1f} s, p95 {np.percentile(done, 95):.1f} s"
        if done
        else "duration: no success"
    )
    causes = Counter(r["cause"] for r in rows if not r["success"])
    return f"{rate}; {duration}; causes: {', '.join(f'{c}: {n}' for c, n in causes.most_common()) or 'none'}"


def xyz(msg: PoseStamped) -> list[float]:
    return [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]


class Bench:
    """The bench's ROS side: /brain_state, the ground truth, the brain's pose, the services."""

    def __init__(self, node):
        self.node = node
        self.states, self.truth, self.pose = [], {}, None
        # the brain computes it from /joint_states: once it comes, /brain/start is not refused for them
        node.create_subscription(PoseStamped, "/end_effector_pose", lambda m: setattr(self, "pose", m), 10)
        latched = QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )  # as /brain_state (01 §3.1)
        node.create_subscription(String, "/brain_state", lambda m: self.states.append(m.data), latched)
        for name in ("object", "drop_box"):
            topic = f"/sim/ground_truth/{name}"
            node.create_subscription(PoseStamped, topic, lambda m, n=name: self.truth.__setitem__(n, m), 10)
        self.services = {
            name: node.create_client(Trigger, name)
            for name in ("/sim/randomize", "/brain/start", "/brain/stop")
        }

    def spin_until(self, condition, timeout_s: float) -> bool:
        end = time.monotonic() + timeout_s
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def call(self, client, request, timeout_s: float = 10.0, resend_s: float | None = None):
        """The reply within timeout_s of the service being found. With resend_s, for a request that changes
        nothing only, it is sent again each resend_s without a reply; any reply counts (self.attempts)."""
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"{client.srv_name}: no such service")
        futures, end = [], time.monotonic() + timeout_s
        while True:
            futures.append(client.call_async(request))
            left = end - time.monotonic()
            self.spin_until(lambda: any(f.done() for f in futures), min(resend_s or left, left))
            replies = [f.result() for f in futures if f.done() and f.result() is not None]
            if replies:
                self.attempts = len(futures)
                return replies[0]
            if resend_s is None or time.monotonic() >= end:
                raise RuntimeError(f"{client.srv_name}: no answer in {timeout_s:g} s")

    def trigger(self, name: str) -> bool:
        return self.call(self.services[name], Trigger.Request()).success

    def driver_seed(self, timeout_s: float) -> int:
        client = self.node.create_client(GetParameters, "/so101_driver/get_parameters")
        reply = self.call(client, GetParameters.Request(names=["seed"]), timeout_s, RESEND_S)
        return reply.values[0].integer_value

    def episode(self, number: int, seed: int, timeout_s: float) -> dict:
        if self.states and self.states[-1].startswith("ERROR"):
            self.trigger("/brain/start")  # re-arm after the previous episode's error
        if not self.spin_until(lambda: self.states and self.states[-1] == "IDLE", 15.0):
            raise RuntimeError(f"/brain_state is {self.states[-1] if self.states else 'silent'}, not IDLE")
        self.trigger("/sim/randomize")
        seen = len(self.states)
        start = time.monotonic()
        ended = lambda: [s for s in self.states[seen:] if s == "DONE" or s.startswith("ERROR")]  # noqa: E731
        if not self.trigger("/brain/start"):
            final = "START_REJECTED"
        elif self.spin_until(ended, timeout_s):
            final = ended()[0]
        else:
            final = "TIMEOUT"
            self.trigger("/brain/stop")
        duration = time.monotonic() - start
        stamp = time.monotonic()
        self.spin_until(
            lambda: time.monotonic() > stamp + 0.3, 1.0
        )  # the ground truth after the cycle (10 Hz)
        cube, box = xyz(self.truth["object"]), xyz(self.truth["drop_box"])
        success, cause = outcome(final, cube, box)
        return {
            "episode": number,
            "seed": seed,
            **{f"cube_{a}": round(v, 4) for a, v in zip("xyz", cube, strict=True)},
            **{f"box_{a}": round(v, 4) for a, v in zip("xyz", box, strict=True)},
            "final_state": final,
            "duration_s": round(duration, 2),
            "success": success,
            "cause": cause,
        }


def record(path: Path) -> subprocess.Popen:
    path.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(path), *BAG_TOPICS],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def run(args, bench: Bench) -> int:
    start = time.monotonic()
    seed = bench.driver_seed(STARTUP_S)
    print(
        f"evaluate: the driver answered in {time.monotonic() - start:.1f} s, request {bench.attempts}",
        flush=True,
    )
    if args.seed is not None and args.seed != seed:
        print(f"evaluate: the driver runs seed {seed}, not {args.seed}", file=sys.stderr)
        return 2
    if not bench.spin_until(lambda: len(bench.truth) == 2, STARTUP_S):
        print(
            "evaluate: no /sim/ground_truth/*: start the driver with expose_ground_truth:=true",
            file=sys.stderr,
        )
        return 2
    if not bench.spin_until(lambda: bench.pose is not None, STARTUP_S):
        print("evaluate: no /end_effector_pose: the brain does not see /joint_states", file=sys.stderr)
        return 2
    rows = []
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        writer = csv.DictWriter(f, FIELDS)
        writer.writeheader()
        for number in range(1, args.episodes + 1):
            bag = args.bags / f"episode_{number:03d}" if args.bags else None
            recorder = record(bag) if bag else None
            row = bench.episode(number, seed, args.timeout)
            if recorder:
                recorder.send_signal(signal.SIGINT)
                recorder.wait(timeout=15)
                if row["success"]:
                    shutil.rmtree(bag, ignore_errors=True)
            writer.writerow(row)
            f.flush()
            rows.append(row)
            print(
                f"episode {number}: {row['final_state']}, {row['duration_s']} s, {row['cause'] or 'success'}"
            )
    print(summary(rows))
    successes = sum(row["success"] for row in rows)
    if args.min_success is not None and successes < args.min_success:
        print(
            f"evaluate: {successes} successes, fewer than --min-success {args.min_success}", file=sys.stderr
        )
        return 1
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--out", type=Path, required=True, help="CSV file, one row per episode")
    parser.add_argument("--bags", type=Path, help="keep a rosbag of each failed episode in this folder")
    parser.add_argument(
        "--seed", type=int, help="the driver's seed parameter, checked before the first episode"
    )
    parser.add_argument("--timeout", type=float, default=60.0, help="bench timeout per episode, s (08 §3)")
    parser.add_argument("--min-success", type=int, help="exit 1 below this many successes (the PR's E2E job)")
    args = parser.parse_args(argv)
    rclpy.init()
    node = rclpy.create_node("evaluate")
    try:
        return run(args, Bench(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
