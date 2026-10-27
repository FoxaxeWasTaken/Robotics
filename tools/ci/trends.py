"""Nightly trends (#60, 08 §2): the IK benchmarks (pytest-benchmark JSON) and the control period (the TREND
line of the driver's launch test) as one CSV row per metric.
Usage: python3 tools/ci/trends.py <ik.json> <control.log> <sha> <out.csv>"""

import datetime
import json
import re
import sys
from pathlib import Path

ik, control, sha, out = sys.argv[1:5]
benchmarks = json.loads(Path(ik).read_text())["benchmarks"]
rows = [(b["name"], 1e3 * b["stats"]["median"], 1e3 * b["stats"]["max"]) for b in benchmarks]  # s -> ms
m = re.search(r"TREND control_period_ms p50=([\d.]+) p99=([\d.]+)", Path(control).read_text())
if not m:
    sys.exit(f"no TREND line in {control}")
rows.append(("control_period", float(m[1]), float(m[2])))
# worst: the max of the IK rounds, the p99 of the period
csv = "date,sha,metric,p50_ms,worst_ms\n" + "".join(
    f"{datetime.date.today()},{sha[:7]},{name},{p50:.3f},{worst:.3f}\n" for name, p50, worst in rows
)
Path(out).write_text(csv)
print(csv)
