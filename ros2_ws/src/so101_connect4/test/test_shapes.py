"""shapes.py: what the game's RViz view draws beside the arm (#280), from the layout, the cells and the
stations as the board reader reads them. No ROS, no sim."""

import numpy as np

from so101_connect4 import board, layout, shapes

LAYOUT = layout.load()
EMPTY = (board.EMPTY,) * 42
TOKEN = LAYOUT["token"]


def drawn(cells=EMPTY, waiting=(0, 0, 0)) -> dict:
    return {(shape.ns, shape.id): shape for shape in shapes.shapes(LAYOUT, cells, waiting)}


def test_an_empty_grid_is_its_volume_and_no_token():
    found = drawn()
    for i, (low, high) in enumerate(layout.grid_boxes(LAYOUT)):
        box = found["grid", i]
        assert (box.frame, box.kind, box.shown) == ("grid", "box", True)
        assert np.allclose(box.R, np.eye(3))
        assert np.allclose(box.center - box.size / 2, low) and np.allclose(box.center + box.size / 2, high)
    tokens = [shape for shape in found.values() if shape.ns == "tokens"]
    assert len(tokens) == 42  # each cell has its shape, hidden: RViz deletes a token at a new game
    assert not any(token.shown for token in tokens)


def test_a_token_read_in_a_cell_stands_there_on_edge_in_its_sides_color():
    cells = list(EMPTY)
    cells[3], cells[7 + 3] = board.HUMAN, board.ROBOT  # column 3: the person's, the robot's on it
    found = drawn(cells)
    person, robot = found["tokens", 3], found["tokens", 10]
    assert person.shown and robot.shown and person.frame == "grid"
    assert np.allclose(person.center, layout.cell(LAYOUT, 3, 0))
    assert np.allclose(robot.center, layout.cell(LAYOUT, 3, 1))
    colors = layout.colors(LAYOUT)
    assert (
        person.color == shapes.RGBA[colors[board.HUMAN]] and robot.color == shapes.RGBA[colors[board.ROBOT]]
    )
    # a disc: its axis, the shape's own z, is across the grid
    assert person.kind == "disc"
    assert np.allclose(person.size, [TOKEN["diameter"], TOKEN["diameter"], TOKEN["thickness"]])
    assert np.allclose(np.abs(person.R[:, 2]), [0.0, 1.0, 0.0])
    assert np.isclose(np.linalg.det(person.R), 1.0)
    assert sum(shape.shown for shape in found.values() if shape.ns == "tokens") == 2


def test_a_token_of_the_robots_color_waits_at_each_station_read_as_taken():
    found = drawn(waiting=(0, 1, 1))
    assert [found["stations", rail].shown for rail in range(3)] == [False, True, True]
    for rail in range(3):
        token = found["stations", rail]
        assert (token.frame, token.kind) == ("dispenser", "disc")
        assert np.allclose(token.center, layout.station(LAYOUT, rail))
        assert token.color == shapes.RGBA[layout.colors(LAYOUT)[board.ROBOT]]


def test_a_dispenser_never_read_shows_no_token():
    assert not any(shape.shown for shape in drawn(waiting=None).values() if shape.ns == "stations")


def test_each_rail_lies_under_its_waiting_tokens_along_its_slope():
    found = drawn()
    along, across = layout.rail_axes(LAYOUT)
    places = LAYOUT["dispenser"]["tokens_per_rail"]
    for rail in range(LAYOUT["dispenser"]["rails"]):
        floor = found["dispenser", rail]
        assert (floor.frame, floor.kind, floor.shown) == ("dispenser", "box", True)
        assert np.allclose(floor.R[:, 0], along) and np.allclose(floor.R[:, 2], across)
        assert np.isclose(np.linalg.det(floor.R), 1.0)
        for place in (0, places - 1):  # the first and the last token stand on it
            offset = layout.waiting(LAYOUT, rail, place) - floor.center
            assert np.isclose(offset @ across, TOKEN["diameter"] / 2 + floor.size[2] / 2)
            assert abs(offset @ along) <= floor.size[0] / 2 - TOKEN["diameter"] / 2 + 1e-9
