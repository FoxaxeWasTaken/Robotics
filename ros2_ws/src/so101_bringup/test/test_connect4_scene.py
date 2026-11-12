"""The Connect Four scene (#79, phase2/P2-02 §7, ADR-036): the grid, its 42 tokens and its markers added to
our copy of the provided scene by sim_scene.generate(connect4=...), from so101_connect4's connect4_grid.yaml.
- The base stays at the origin (MuJoCo world = base_link), the table is one riser down.
- The camera of phase 1, unmoved, sees the 4 markers where layout.py puts them.
- Tokens fall into their column and stack; the arm at a release pose touches nothing of the grid.
The ground truth (token poses, the camera's pose) is read here to score, as a test may (C-06)."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_brain.drop import release_pose
from so101_brain.kinematics import PAN_AXIS_X, Kinematics
from so101_bringup import sim_scene
from so101_connect4 import layout
from so101_driver.camera import opencv_intrinsics
from so101_perception import calibration

CAMERA = yaml.safe_load(
    (Path(get_package_share_directory("so101_bringup")) / "config" / "sim_scene.yaml").read_text()
)["camera"]
URDF = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
LAYOUT = layout.load()
GRID, TOKEN = LAYOUT["grid"], LAYOUT["token"]
T_GRID = layout.grid_pose(LAYOUT)
HANDOVER = [-1.0, -0.6, 0.6, 1.1, 1.57]  # the candidate of P2-02 §7: the reader reads with the arm there
FLIP = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera frame (y up, looks along -z) -> OpenCV optical frame
ARM_BODIES = ["shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    return sim_scene.generate(tmp_path_factory.mktemp("scene"), camera=CAMERA, connect4=LAYOUT)


@pytest.fixture
def sim(scene):
    """A fresh sim on the game scene, the arm at HANDOVER."""
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480)
    sim.connect()
    set_arm(sim, HANDOVER)
    yield sim
    sim.disconnect()


def set_arm(sim, q):
    for name, value in zip(ARM_JOINTS, q, strict=False):
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = value
        sim.data.ctrl[sim.model.actuator(name).id] = value
    mujoco.mj_forward(sim.model, sim.data)


def in_base(point_in_grid):
    return (T_GRID @ [*point_in_grid, 1.0])[:3]


def place(sim, token, column, height_over_entry=0.02):
    """Teleport a token above a column, its disc in the slot plane, at rest."""
    joint = sim.model.joint(f"{token}_free")
    q, v = joint.qposadr[0], joint.dofadr[0]
    sim.data.qpos[q : q + 3] = in_base(layout.cell(LAYOUT, column, GRID["rows"]))
    sim.data.qpos[q + 2] += height_over_entry
    quat = np.zeros(4)  # the disc's axis (its z) along the grid's normal (y)
    mujoco.mju_mat2Quat(quat, (T_GRID[:3, :3] @ np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])).ravel())
    sim.data.qpos[q + 3 : q + 7] = quat
    sim.data.qvel[v : v + 6] = 0
    mujoco.mj_forward(sim.model, sim.data)


def cell_of(sim, token):
    """(column, row) as floats, and the offset across the slot (m), from the token's true pose."""
    p = np.linalg.inv(T_GRID) @ [*sim.data.body(token).xpos, 1.0]
    return p[0] / GRID["column_pitch"], p[2] / GRID["row_pitch"], p[1]


def step(sim, seconds):
    for _ in range(round(seconds / sim.model.opt.timestep)):
        mujoco.mj_step(sim.model, sim.data)


def test_the_base_stays_at_the_origin_and_the_table_is_one_riser_down(sim):
    assert np.allclose(sim.data.body("base").xpos, 0.0, atol=1e-9)  # world = base_link (driver_node)
    assert np.isclose(sim.model.geom("floor").pos[2], -LAYOUT["placement"]["riser"])
    assert LAYOUT["placement"]["pan_axis"] == [PAN_AXIS_X, 0.0]


def test_the_grid_stands_where_the_layout_puts_it(sim):
    assert np.allclose(sim.data.body("grid").xpos, T_GRID[:3, 3], atol=1e-9)
    assert np.allclose(sim.data.body("grid").xmat.reshape(3, 3), T_GRID[:3, :3], atol=1e-6)


def test_42_tokens_half_red_half_yellow_wait_on_the_table_out_of_the_cameras_view(sim):
    names = [f"token_{color}_{i}" for color in ("red", "yellow") for i in range(21)]
    T = sim.get_camera_extrinsics()  # test only
    R, t = T[:3, :3] @ FLIP, T[:3, 3]
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    step(sim, 0.5)
    table = -LAYOUT["placement"]["riser"]
    for name in names:
        position = sim.data.body(name).xpos
        assert np.isclose(position[2], table + TOKEN["thickness"] / 2, atol=1e-3), name  # flat on the table
        assert np.hypot(position[0] - PAN_AXIS_X, position[1]) > 0.40, name  # out of the arm's reach
        camera = (position - t) @ R
        u, v = (camera @ K.T)[:2] / camera[2]
        assert not (camera[2] > 0 and -20 < u < 660 and -20 < v < 500), (
            name,
            u,
            v,
        )  # nor a blob in the image
    assert sum(sim.model.body(i).name.startswith("token_") for i in range(sim.model.nbody)) == 42


def test_the_phase_1_camera_sees_the_four_markers_where_the_layout_puts_them(sim):
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    renderer.update_scene(sim.data, camera=sim.camera_name)
    found = calibration.detect_markers(renderer.render(), LAYOUT["markers"]["dictionary"])
    renderer.close()
    assert sorted(found) == sorted(LAYOUT["markers"]["ids"])
    T = sim.get_camera_extrinsics()  # test only: the camera's pose, to check the layout's convention
    R, t = T[:3, :3] @ FLIP, T[:3, 3]
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    for i, detected in found.items():
        corners = np.array([in_base(c) for c in layout.marker_corners(LAYOUT, i)])
        camera = (corners - t) @ R
        uv = (camera @ K.T)[:, :2] / camera[:, 2:]
        # 8 px as the calibration markers (MES-54); a wrong order or a wrong face: a side, 30 px or more
        assert np.abs(uv - detected).max() < 8.0, (i, uv.round(1).tolist(), detected.round(1).tolist())


def test_a_token_dropped_above_each_column_lands_in_its_bottom_row(sim):
    for column in range(GRID["columns"]):
        place(sim, f"token_red_{column}", column)
    step(sim, 1.5)
    for column in range(GRID["columns"]):
        at, row, across = cell_of(sim, f"token_red_{column}")
        assert abs(at - column) < 0.1 and abs(row) < 0.1, (column, at, row)
        assert abs(across) < GRID["slot_gap"] / 2


def test_six_tokens_stack_in_a_column_and_come_to_rest(sim):
    for row in range(GRID["rows"]):
        place(sim, f"token_yellow_{row}", 3)
        step(sim, 1.0)
    step(sim, 1.0)
    for row in range(GRID["rows"]):
        at, height, _ = cell_of(sim, f"token_yellow_{row}")
        assert abs(at - 3) < 0.1 and abs(height - row) < 0.1, (row, at, height)
        velocity = sim.data.cvel[sim.model.body(f"token_yellow_{row}").id]
        assert np.abs(velocity).max() < 1e-3, (row, velocity)  # the provided contacts crept at 70 mm/s


def test_the_phase_1_cube_is_a_ghost_and_the_box_is_out_of_the_way(sim):
    cube = sim.model.body("pickup_object")
    geoms = [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == cube.id]
    assert all(sim.model.geom_contype[g] == 0 and sim.model.geom_conaffinity[g] == 0 for g in geoms)
    assert all(sim.model.geom_rgba[g][3] == 0 or sim.model.geom_group[g] >= 3 for g in geoms)  # not drawn
    before = sim.data.body("pickup_object").xpos.copy()
    step(sim, 1.0)
    assert np.allclose(sim.data.body("pickup_object").xpos, before, atol=1e-3)  # it does not fall
    assert np.linalg.norm(sim.data.body("drop_box").xpos[:2]) > 1.0


def test_the_arm_touches_nothing_of_the_grid_at_the_release_of_each_column(sim):
    """The frame and its posts leave the drop poses free: release and pre-drop (25 mm up, P2-02 §2.3)."""
    kin = Kinematics(str(URDF))
    grid = sim.model.body("grid").id
    grid_geoms = [  # the markers are plane geoms, endless for a distance: they lie on the posts anyway
        g
        for g in range(sim.model.ngeom)
        if sim.model.geom_bodyid[g] == grid and sim.model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE
    ]
    arm = {sim.model.body(name).id for name in ARM_BODIES}
    arm_geoms = [
        g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] in arm and sim.model.geom_contype[g]
    ]
    assert arm_geoms and grid_geoms
    above_guide = (
        GRID["depth_guide_height"] + 0.008 + TOKEN["diameter"] / 2
    )  # the token's bottom 8 mm over it
    closest = {}
    for column in range(GRID["columns"]):
        target = in_base(layout.cell(LAYOUT, column, GRID["rows"]))
        target += 0.001 * T_GRID[:3, 1]  # aimed 1 mm towards the fixed jaw (P2-02 §2.2)
        target[2] += -GRID["row_pitch"] / 2 + above_guide
        for lift in (0.0, 0.025):
            q = release_pose(kin, [target[0], target[1], target[2] + lift])
            assert q is not None, (column, lift)
            set_arm(sim, q)
            gaps = [
                mujoco.mj_geomDistance(sim.model, sim.data, a, g, 0.05, None)
                for a in arm_geoms
                for g in grid_geoms
            ]
            pair = int(np.argmin(gaps))
            names = (
                sim.model.body(sim.model.geom_bodyid[arm_geoms[pair // len(grid_geoms)]]).name,
                sim.model.geom(grid_geoms[pair % len(grid_geoms)]).name,
            )
            closest[column, lift] = (round(min(gaps) * 1e3, 1), *names)
    print("arm to grid, mm:", closest)
    assert min(gap for gap, *_ in closest.values()) > 5.0, closest


def test_without_connect4_the_scene_has_no_grid_and_no_token(tmp_path):
    xml = sim_scene.generate(tmp_path, camera=CAMERA).read_text()
    assert "grid" not in xml and "token" not in xml
