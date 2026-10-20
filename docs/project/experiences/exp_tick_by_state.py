"""The brain's tick period per FSM state, to tell what the planning worker costs the tick from the load of the
machine (step 4.2b, C-33). Feeds MES-40. Works on a COPY of the workspace, never on the package.

    python3 exp_tick_by_state.py patch /tmp/ws/src/so101_brain/so101_brain/brain_node.py [--thread]
    # build the copy, then run the cycles of exp_gt_cycles.py on it
    python3 exp_tick_by_state.py report /tmp/ticks.csv

patch: every tick writes "state,steady time" to /tmp/ticks.csv. --thread: the planner back in a worker thread
(the first design of 4.2b), which shares the GIL with the tick, instead of the node's worker process.
"""

import collections
import sys
from pathlib import Path

import numpy as np


def sub(s, old, new):
    assert s.count(old) == 1, old[:60]
    return s.replace(old, new)


def patch(path, thread):
    p = Path(path)
    s = p.read_text()
    old = "        now, planning = self._now(), self._fsm.state is State.PLAN\n"
    s = sub(s, old, old + "        self._dump.write(f'{self._fsm.label},{now}\\n')\n")
    old = "        self._last_tick = None\n"
    s = sub(s, old, old + "        self._dump = open('/tmp/ticks.csv', 'a', buffering=1)\n")
    if thread:  # the first design of 4.2b, for comparison: the same planner in a worker thread
        s = sub(s, "        self._worker = planner_pool(str(urdf), self._declare_planner())  # its own process (06 §4)\n", """\
        global _PLANNER
        _PLANNER = GraspPlanner(Kinematics(str(urdf)), **self._declare_planner())
        from concurrent.futures import ThreadPoolExecutor
        self._worker = ThreadPoolExecutor(max_workers=1)
""")
    p.write_text(s)


def report(path):
    rows = [line.strip().split(",") for line in open(path)]
    per = collections.defaultdict(list)
    for (_, t0), (state, t1) in zip(rows, rows[1:], strict=False):
        per[state].append((float(t1) - float(t0)) * 1000)
    per["ALL"] = [v for values in list(per.values()) for v in values]
    for state in ("ALL", "IDLE", "WAIT_TARGET", "PLAN", "APPROACH", "TRANSPORT", "HOME"):
        v = np.array(per[state])
        if len(v):
            print(f"{state:12s} {len(v):5d} ticks: p50 {np.percentile(v, 50):5.1f}, p99 {np.percentile(v, 99):5.1f}, "
                  f"max {v.max():5.1f} ms; over 25 ms: {(v > 25).mean() * 100:5.2f} %")


if __name__ == "__main__":
    if sys.argv[1] == "patch":
        patch(sys.argv[2], "--thread" in sys.argv)
    else:
        report(sys.argv[2])
