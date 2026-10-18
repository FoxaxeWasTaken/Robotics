"""Drop-box position sampler of /sim/randomize (docs/project/03 §7, ADR-018). Pure, no ROS.

The cube keeps the provided randomize_object(); only the box needs a sampler (C-42).
"""

import numpy as np

# Area of the box center, from the base_link origin: reachable at drop height (04 §4) and the whole box
# in the view of the scene's camera, with a 5 px margin (test_sim_tools.py). Positive azimuths lie close
# to the camera: there the bottom edge of the image cuts the box.
BOX_RADIUS = (0.18, 0.30)  # m
BOX_AZIMUTH = (np.radians(-90.0), np.radians(-10.0))
# Between the box and cube centers: box half-diagonal 7.4 cm + cube half-diagonal 3.5 cm + 4 cm for the
# open jaw grasping the cube next to a wall.
MIN_CUBE_DISTANCE = 0.15  # m


def sample_box_xy(rng: np.random.Generator, cube_xy) -> np.ndarray:
    """Box center (x, y) in the area, at least MIN_CUBE_DISTANCE from the cube.

    Always ends: no disc of that radius covers the area.
    """
    cube_xy = np.asarray(cube_xy, dtype=float)[:2]
    while True:
        r, azimuth = rng.uniform(*BOX_RADIUS), rng.uniform(*BOX_AZIMUTH)
        box = np.array([r * np.cos(azimuth), r * np.sin(azimuth)])
        if np.linalg.norm(box - cube_xy) >= MIN_CUBE_DISTANCE:
            return box
