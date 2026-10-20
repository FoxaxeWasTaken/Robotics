"""Unit test of the sim camera's K in the OpenCV pixel convention (docs/project/05 §3.2, MES-41), checked
against the renders: the cube's red blob against its exact silhouette, projected with that K."""

import cv2
import mujoco
import numpy as np
import pytest
from so101_sim import SO101Sim

from so101_driver.camera import opencv_intrinsics

FLIP = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera (y up, looks along -z) -> OpenCV optical (05 §3.1)
CORNERS = 0.025 * np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])  # 5 cm cube


@pytest.fixture(scope="module")
def sim():
    sim = SO101Sim(seed=0)
    sim.connect()
    m = sim.model
    m.body_pos[m.body("shoulder").id] += [0.0, 0.0, 10.0]  # the arm out of the picture: no occlusion
    yield sim
    sim.disconnect()


def test_the_cube_blob_is_centered_on_its_silhouette_projected_with_the_published_k(sim):
    """Mean offset within 0.25 px in u and v: (-0.03, -0.13) px measured, (-0.53, -0.63) px with the
    provided K, whose principal point (W/2, H/2) is a pixel corner in OpenCV coordinates."""
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    T = sim.get_camera_extrinsics()
    R, t = T[:3, :3] @ FLIP, T[:3, 3]
    height, width = sim.camera_height, sim.camera_width
    offsets = []
    while len(offsets) < 10:
        sim.randomize_object()
        mujoco.mj_forward(sim.model, sim.data)
        cube = sim.data.body("pickup_object")
        hsv = cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2HSV)
        red = cv2.inRange(hsv, (160, 100, 60), (180, 255, 255))
        red |= cv2.inRange(hsv, (0, 100, 60), (8, 255, 255))  # red wraps around H = 0 (05 §2.2)
        rows, cols = np.nonzero(red)
        if rows.min() == 0 or cols.min() == 0 or rows.max() == height - 1 or cols.max() == width - 1:
            continue  # cut by the border: so is the silhouette
        corners = (cube.xpos + CORNERS @ cube.xmat.reshape(3, 3).T - t) @ R @ K.T
        hull = cv2.convexHull((corners[:, :2] / corners[:, 2:]).astype(np.float32))
        m = cv2.moments(hull)
        offsets.append([cols.mean() - m["m10"] / m["m00"], rows.mean() - m["m01"] / m["m00"]])
    mean = np.mean(offsets, axis=0)
    print(f"blob - silhouette centroid: mean ({mean[0]:+.3f}, {mean[1]:+.3f}) px over {len(offsets)}")
    assert np.all(np.abs(mean) < 0.25)
