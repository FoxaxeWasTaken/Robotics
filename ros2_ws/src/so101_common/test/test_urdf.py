"""so101_common must match the URDF of so101_description: a typo here would clamp or name a joint wrongly."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from so101_common.robot import BASE_FRAME, JOINT_LIMITS, JOINT_NAMES, TCP_FRAME

URDF = ET.parse(Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf").getroot()
REVOLUTE = {j.get("name"): j for j in URDF.iter("joint") if j.get("type") == "revolute"}


def test_limits_match_urdf():
    assert set(JOINT_LIMITS) == set(REVOLUTE)
    for name, (lo, hi) in JOINT_LIMITS.items():
        limit = REVOLUTE[name].find("limit")
        assert lo == pytest.approx(float(limit.get("lower")), abs=1e-5), name
        assert hi == pytest.approx(float(limit.get("upper")), abs=1e-5), name


def test_joint_names_follow_the_urdf_chain_from_the_base():
    by_parent = {j.find("parent").get("link"): j for j in REVOLUTE.values()}
    chain, link = [], BASE_FRAME
    while link in by_parent:
        chain.append(by_parent[link].get("name"))
        link = by_parent[link].find("child").get("link")
    assert tuple(chain) == JOINT_NAMES


def test_frames_are_urdf_links():
    assert {BASE_FRAME, TCP_FRAME} <= {link.get("name") for link in URDF.iter("link")}
