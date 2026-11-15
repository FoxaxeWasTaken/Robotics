#!/usr/bin/env python3
"""check_scad_dimensions.py: the printable parts have the simulation's dimensions.

Every value of hardware/connect4/dimensions.scad (mm, degrees) is the one of connect4_grid.yaml (m, rad) at
the path its name spells. pre-commit runs it when either file changes. Usage: python3
tools/ci/check_scad_dimensions.py
"""

import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
YAML = ROOT / "ros2_ws/src/so101_connect4/config/connect4_grid.yaml"
SCAD = ROOT / "hardware/connect4/dimensions.scad"
COUNTS = ("columns", "rows", "rails", "tokens_per_rail")


def layout() -> dict:
    """{a value's path, its keys joined by "_": its text}. The file is maps of scalars and inline lists: read
    by hand, the lint job's Python has no PyYAML."""
    values, stack = {}, []
    for line in YAML.read_text().splitlines():
        found = re.match(r"( *)(\w+):\s*(.*)", line.split("#")[0].rstrip())
        if found:
            indent, key, value = len(found[1]), found[2], found[3]
            stack = [(i, k) for i, k in stack if i < indent] + [(indent, key)]
            if value:
                values["_".join(k for _, k in stack)] = value
    return values


def in_scad_units(name, text):
    """A YAML value as dimensions.scad writes it: a count as it is, the slope in degrees, a length in mm."""
    if text.startswith("["):
        return [in_scad_units(name, item) for item in text.strip("[]").split(",")]
    if name.endswith(COUNTS):
        return float(text)
    return math.degrees(float(text)) if name.endswith("slope") else float(text) * 1000


def main() -> int:
    values, errors = layout(), []
    for name, text in re.findall(r"^(\w+) = (.+?);", SCAD.read_text(), re.M):
        if name not in values:
            errors.append(f"{name}: no such value in {YAML.name}")
            continue
        want, got = in_scad_units(name, values[name]), in_scad_units("columns", text)
        pairs = zip(want, got, strict=True) if isinstance(want, list) else [(want, got)]
        if any(abs(a - b) > 0.01 for a, b in pairs):
            errors.append(f"{name} = {text}, the YAML says {want}")
    for error in errors:
        print(f"check_scad_dimensions: {error}", file=sys.stderr)
    print(f"ok: {SCAD.name}'s values are {YAML.name}'s" if not errors else "FAIL")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
