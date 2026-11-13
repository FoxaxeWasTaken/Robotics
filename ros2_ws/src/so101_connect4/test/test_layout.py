"""layout.py: the geometry every user of connect4_grid.yaml shares (phase2/P2-02 §6, §7). No ROS, no sim."""

from itertools import pairwise

import numpy as np
import pytest
import yaml

from so101_connect4 import layout

LAYOUT = layout.load()


def test_the_grid_frame_is_at_the_first_cell_x_along_the_columns_z_up():
    T = layout.grid_pose(LAYOUT)
    azimuth = np.radians(-40.0)
    assert np.allclose(T[:3, 0], [np.cos(azimuth), np.sin(azimuth), 0.0], atol=1e-4)
    assert np.allclose(T[:3, 2], [0.0, 0.0, 1.0])
    assert np.allclose(np.cross(T[:3, 2], T[:3, 0]), T[:3, 1])  # y = z cross x
    # column 0 is 0.14 m from the pan axis, the slot plane 3.85 mm off it along -y
    from_axis = T[:2, 3] - LAYOUT["placement"]["pan_axis"]
    assert np.isclose(from_axis @ T[:2, 0], 0.14)
    assert np.isclose(from_axis @ T[:2, 1], -0.00385)


def test_six_stacked_tokens_end_just_under_the_slot_entry():
    T = layout.grid_pose(LAYOUT)
    top_row = (T @ [*layout.cell(LAYOUT, 3, 5), 1.0])[:3]
    entry = -0.10 + 0.25  # the table one riser under base_link, the entries 25 cm above it
    assert np.isclose(top_row[2] + LAYOUT["token"]["diameter"] / 2, entry - 0.002)
    assert LAYOUT["grid"]["row_pitch"] == LAYOUT["token"]["diameter"]  # tokens rest on each other


def test_the_last_column_is_6_pitches_out():
    assert np.allclose(layout.cell(LAYOUT, 6, 0), [6 * 0.037, 0.0, 0.0])


def test_the_markers_sit_on_the_front_face_of_the_posts_between_the_table_and_the_entry():
    centers = layout.marker_centers(LAYOUT)
    assert sorted(centers) == [10, 11, 12, 13]
    half = LAYOUT["markers"]["plate"] / 2
    table = -LAYOUT["grid"]["row_pitch"] / 2 - layout.slot_bottom(LAYOUT)
    entry = table + LAYOUT["grid"]["slot_entry_height"]
    for center in centers.values():
        assert center[1] > LAYOUT["grid"]["slot_gap"] / 2  # the front face, +y
        assert table < center[2] - half
        assert center[2] + half < entry
        on_a_post = center[0] + half < -0.037 / 2 or center[0] - half > 6.5 * 0.037
        assert on_a_post  # never over a window
    assert centers[10][0] < 0 < centers[11][0]
    assert centers[10][2] > centers[12][2]


def test_marker_corners_follow_the_order_cv2_aruco_detects_seen_from_the_front():
    corners = layout.marker_corners(LAYOUT, 10)
    top_left, top_right, _, bottom_left = corners
    assert top_left[0] > top_right[0]  # the viewer's right is -x
    assert top_left[2] > bottom_left[2]
    assert np.isclose(top_right[2], top_left[2])
    assert np.allclose(corners.mean(axis=0), layout.marker_centers(LAYOUT)[10])
    assert np.isclose(np.linalg.norm(top_left - top_right), LAYOUT["markers"]["size"])


def robot_on(color) -> dict:
    return {**LAYOUT, "token": {**LAYOUT["token"], "robot_color": color}}


def test_the_robot_plays_yellow_in_the_installed_file_and_the_person_the_other_color():
    """#255: one place pairs a side (1 the person's, 2 the robot's) with a color."""
    assert LAYOUT["token"]["robot_color"] == "yellow"
    assert layout.colors(LAYOUT) == {1: "red", 2: "yellow"}


def test_the_robots_color_is_a_setting_of_the_file():
    assert layout.colors(robot_on("red")) == {1: "yellow", 2: "red"}


def test_a_color_the_reader_has_no_range_for_is_refused_when_the_file_is_loaded(tmp_path):
    path = tmp_path / "connect4_grid.yaml"
    path.write_text(yaml.safe_dump(robot_on("blue")))
    with pytest.raises(ValueError, match=r"robot_color.*blue.*red.*yellow"):
        layout.load(path)


# The token dispenser (#257, ADR-041): three straight rails stacked in the arm's plane, each with a station.


def rail_axes():
    slope = LAYOUT["dispenser"]["slope"]
    return np.array([np.cos(slope), 0.0, np.sin(slope)]), np.array([-np.sin(slope), 0.0, np.cos(slope)])


def test_the_dispenser_holds_the_tokens_of_one_side_of_a_game():
    dispenser, grid = LAYOUT["dispenser"], LAYOUT["grid"]
    assert dispenser["rails"] * dispenser["tokens_per_rail"] == grid["columns"] * grid["rows"] // 2


def test_the_dispensers_frame_is_at_its_lowest_station_x_outwards_in_the_arms_plane():
    T, place = layout.dispenser_pose(LAYOUT), LAYOUT["dispenser"]["placement"]
    azimuth = place["azimuth"]
    assert np.allclose(T[:3, 0], [np.cos(azimuth), np.sin(azimuth), 0.0])
    assert np.allclose(T[:3, 2], [0.0, 0.0, 1.0])
    assert np.allclose(np.cross(T[:3, 2], T[:3, 0]), T[:3, 1])
    from_axis = T[:2, 3] - LAYOUT["placement"]["pan_axis"]
    assert np.isclose(from_axis @ T[:2, 0], place["station_r"])
    assert np.isclose(from_axis @ T[:2, 1], -place["plane_offset"])  # the tokens' plane, as the grid's slots
    assert np.isclose(T[2, 3], place["station_z"])
    assert np.allclose(layout.station(LAYOUT, 0), 0.0)


def test_the_stations_climb_like_stairs_outwards_one_rail_pitch_above_the_rail_below():
    dispenser = LAYOUT["dispenser"]
    _, normal = rail_axes()
    for rail in range(1, dispenser["rails"]):
        step = layout.station(LAYOUT, rail) - layout.station(LAYOUT, rail - 1)
        assert np.isclose(step[0], dispenser["station_step"])  # outwards: the gripper clears the rail above
        assert step[1] == 0.0  # one plane
        assert np.isclose(step @ normal, dispenser["rail_pitch"])


def test_the_tokens_of_a_rail_wait_rim_to_rim_up_its_slope_from_its_station():
    dispenser = LAYOUT["dispenser"]
    along, _ = rail_axes()
    for rail in range(dispenser["rails"]):
        places = [layout.waiting(LAYOUT, rail, i) for i in range(dispenser["tokens_per_rail"])]
        assert np.allclose(places[0], layout.station(LAYOUT, rail))
        for lower, upper in pairwise(places):
            assert np.allclose(upper - lower, LAYOUT["token"]["diameter"] * along)


def test_a_rail_passes_over_the_tokens_of_the_rail_below():
    dispenser, token = LAYOUT["dispenser"], LAYOUT["token"]
    assert dispenser["rail_pitch"] >= token["diameter"] + dispenser["floor"] + 0.002


def test_a_token_stands_in_the_slot_with_the_play_of_the_grids_slots():
    dispenser, grid = LAYOUT["dispenser"], LAYOUT["grid"]
    assert dispenser["slot_gap"] == grid["slot_gap"]
    assert (
        dispenser["station_wall_height"] <= 0.009
    )  # the open jaws go 5 mm under the token's centre [MES-75]
    assert dispenser["stop_width"] < LAYOUT["token"]["thickness"]  # a fin between the jaws
