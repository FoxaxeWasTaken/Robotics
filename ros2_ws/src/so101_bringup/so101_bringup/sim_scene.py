"""sim_scene - our sim scenes: a copy of the provided scene edited with mujoco.MjSpec, never sim/ (ADR-036).

generate() copies the provided scene folder (its include and mesh paths resolve from the main file), edits the
copy and writes the result next to it; the driver loads it through its scene_path parameter. SimScene is the
launch substitution that does it at launch time, in sim only.

- camera: the pose of the external camera, config/sim_scene.yaml (#47): the provided one cut 21 % of the
  cubes at the image edge [MES-12];
- markers: the ArUco markers of config/markers_sim.yaml on the table, for the extrinsic calibration
  (3.4a, #44);
- lighting: every light, the headlight included, darker and tinted (config/sim_scene.yaml), for the robustness
  demo's lighting case (#103);
- connect4: the Connect Four game of phase 2 (#79, phase2/P2-02 §7), from so101_connect4's connect4_grid.yaml:
  the table one riser down (the base stays at the origin, so MuJoCo world = base_link), the grid with its
  depth guide and its markers, 42 tokens, the contact set the drop spike measured [MES-63]; the cube of
  phase 1 a ghost, its box away.
"""

import shutil
import struct
import tempfile
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitution import Substitution
from launch.utilities import perform_substitutions
from so101_sim import DEFAULT_SCENE

from so101_common.markers import marker_corners  # noqa: F401 (the convention the textures follow)
from so101_connect4 import layout as connect4_layout

MARKER_PIXELS = 600  # texture resolution of a marker's black square


def generate(folder, camera=None, markers=None, lighting=None, connect4=None) -> Path:
    """Our scene in folder (a copy of the provided one, edited); returns the path of its main file."""
    folder = Path(folder) / "so101"
    shutil.copytree(Path(DEFAULT_SCENE).parent, folder)
    spec = mujoco.MjSpec.from_file(str(folder / Path(DEFAULT_SCENE).name))
    if camera:
        cam = spec.camera("external_cam")
        cam.pos = camera["pos"]
        cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES  # the provided camera uses xyaxes, which
        cam.alt.xyaxes = camera["xyaxes"]  # a quat set alone would not override
    if markers:
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, markers["dictionary"]))
        border = round(MARKER_PIXELS * (markers["plane_m"] - markers["side_m"]) / markers["side_m"] / 2)
        half = markers["plane_m"] / 2
        for i, (x, y) in markers["markers"].items():
            image = cv2.aruco.generateImageMarker(dictionary, i, MARKER_PIXELS)
            image = cv2.copyMakeBorder(image, *(border,) * 4, cv2.BORDER_CONSTANT, value=255)  # quiet zone
            cv2.imwrite(str(folder / f"aruco_{i}.png"), np.rot90(image, -1))  # see so101_common.markers
            spec.add_texture(name=f"aruco_{i}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=f"aruco_{i}.png")
            material = spec.add_material(name=f"aruco_{i}")
            material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = f"aruco_{i}"
            spec.worldbody.add_geom(
                name=f"aruco_{i}",
                type=mujoco.mjtGeom.mjGEOM_PLANE,
                size=[half, half, 0.001],
                pos=[x, y, 0.0005],  # just above the floor: no z-fighting
                material=f"aruco_{i}",
                contype=0,
                conaffinity=0,
            )
    if lighting:
        factor = lighting["scale"] * np.asarray(lighting["tint"], dtype=float)
        for light in [*spec.lights, spec.visual.headlight]:
            light.diffuse = np.asarray(light.diffuse) * factor
            light.ambient = np.asarray(light.ambient) * factor
            light.specular = np.asarray(light.specular) * factor
    if connect4:
        _add_connect4(spec, folder, connect4)
    path = folder / "scene_so101.xml"
    path.write_text(spec.to_xml())
    return path


BLUE, RED, YELLOW = [0.06, 0.20, 0.72, 1], [0.80, 0.05, 0.10, 1], [0.95, 0.78, 0.05, 1]
BOX = mujoco.mjtGeom.mjGEOM_BOX
RISER_HALF = 0.07  # m, the block under the base
PARKING = (-0.25, 0.35)  # m, base_link: the first spare token, on the table out of the camera's view


def token_parking(color, i, table_z, thickness) -> list:
    """Where a spare token lies flat on the table, 4 cm from the next: two rows per color, out of the arm's
    reach and of the camera's view (a token in view is a red or yellow blob beside the grid)."""
    row = i // 11 + 2 * (color == "yellow")
    return [PARKING[0] - 0.04 * (i % 11), PARKING[1] + 0.04 * row, table_z + thickness / 2]


def _add_connect4(spec, folder, layout):
    grid, place, token, contact = layout["grid"], layout["placement"], layout["token"], layout["contact"]
    table_z = -place["riser"]
    # The table goes down, the base stays: world = base_link, the extrinsics and the FK tests hold [MES-65].
    spec.geom("floor").pos[2] = table_z
    spec.worldbody.add_geom(
        name="riser",
        type=BOX,
        size=[RISER_HALF, RISER_HALF, place["riser"] / 2],
        pos=[0.01, 0, table_z / 2],
        rgba=[0.3, 0.3, 0.3, 1],
    )
    # connect() needs the cube's free joint (randomize_object): a ghost, so nothing parks it at run time.
    spec.body("drop_box").pos = [2.0, 0.0, table_z]
    cube = spec.body("pickup_object")
    spec.delete(spec.geom("pickup_object_visual"))
    for geom in cube.geoms:
        geom.contype = geom.conaffinity = 0
        geom.group = 3  # never drawn
    cube.gravcomp = 1.0
    # The contact set [MES-63]: the provided one lets a token creep, and the jaws squeeze through it.
    spec.option.timestep = contact["timestep"]
    spec.option.impratio = contact["impratio"]
    spec.option.cone = getattr(mujoco.mjtCone, f"mjCONE_{contact['cone'].upper()}")
    spec.option.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP
    spec.option.sleep_tolerance = contact["sleep_tolerance"]
    # OSMesa: 183 ms a frame as provided, 27 ms without shadows and reflection [MES-63]
    spec.visual.quality.shadowsize = 0
    spec.material("groundplane").reflectance = 0
    solid = {
        "friction": [token["friction"], 0.005, 0.0001],
        "solref": [contact["solref"], 1],
        "margin": 0,
        "condim": 3,
    }
    _add_grid(spec, folder, layout, solid)
    _add_dispenser(spec, folder, layout, table_z, solid)
    robot = connect4_layout.colors(layout)[2]
    for color, rgba in (("red", RED), ("yellow", YELLOW)):
        for i in range(grid["columns"] * grid["rows"] // 2):
            name = f"token_{color}_{i}"
            parked = token_parking(color, i, table_z, token["thickness"])
            if color == robot:  # the robot's wait in the dispenser's rails, the person's lie on the table
                pos, quat = _waiting_pose(layout, i)
                # where the simulated person lays it when it empties the dispenser (connect4_sim)
                spec.worldbody.add_site(name=f"spare_{name}", pos=parked, size=[0.001] * 3, rgba=[0, 0, 0, 0])
            else:
                pos, quat = parked, [1, 0, 0, 0]
            body = spec.worldbody.add_body(name=name, pos=pos, quat=quat)
            body.add_joint(name=f"{name}_free", type=mujoco.mjtJoint.mjJNT_FREE).damping = [
                contact["damping"],
                0,
                0,
            ]
            body.add_geom(
                name=name,
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                size=[token["diameter"] / 2, token["thickness"] / 2, 0],
                mass=token["mass"],
                rgba=rgba,
                solmix=contact["solmix"],
                **solid,
            )


def _waiting_pose(layout, i) -> tuple:
    """(position, quaternion) in base_link of the robot's token `i` in the dispenser: on edge, its disc in the
    rails' plane, at layout.waiting."""
    per_rail = layout["dispenser"]["tokens_per_rail"]
    T = connect4_layout.dispenser_pose(layout)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, (T[:3, :3] @ [[1.0, 0, 0], [0, 0, 1], [0, -1, 0]]).ravel())  # the disc's z on y
    return (T @ [*connect4_layout.waiting(layout, i // per_rail, i % per_rail), 1.0])[:3], quat


def _add_dispenser(spec, folder, layout, table_z, solid):
    """The token dispenser, in its frame (so101_connect4.layout, ADR-041): for each rail a floor, two side
    walls, low at the pick station and high behind it, and a stop, a fin in the tokens' plane."""
    dispenser, token = layout["dispenser"], layout["token"]
    radius, diameter = token["diameter"] / 2, token["diameter"]
    T = connect4_layout.dispenser_pose(layout)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, T[:3, :3].ravel())
    body = spec.worldbody.add_body(name="dispenser", pos=T[:3, 3], quat=quat)
    along, across = connect4_layout.rail_axes(layout)
    tilt = np.zeros(4)  # a rail's boxes: x up the rail, y across the slot, z off its floor
    mujoco.mju_mat2Quat(tilt, np.column_stack([along, [0.0, 1.0, 0.0], across]).ravel())
    gap, wall, stop = dispenser["slot_gap"], dispenser["wall"], 0.003
    start = -radius - stop  # the stop's back, along the rail from the station
    # Where the low walls of the station end: a token lifted straight up moves up the sloped rail, and it is
    # above the high walls only once it has swept their height times the slope's tangent. Ending 3 mm behind
    # the token, a token held 1.5 mm off the slot's middle rubbed the high wall and stopped the lift [MES-77].
    cut = radius + dispenser["rail_wall_height"] * np.tan(dispenser["slope"]) + 0.002
    end = (dispenser["tokens_per_rail"] - 1) * diameter + radius + 0.004
    part = {"type": BOX, "rgba": [0.85, 0.85, 0.9, 1], "quat": tilt, **solid}

    def box(name, rail, x, y, z, half_y):
        """A box from x[0] to x[1] up the rail and from z[0] to z[1] off the token's centre, at y across."""
        centre = connect4_layout.station(layout, rail) + sum(x) / 2 * along + sum(z) / 2 * across
        centre[1] = y
        size = [(x[1] - x[0]) / 2, half_y, (z[1] - z[0]) / 2]
        body.add_geom(name=f"dispenser_{name}_{rail}", pos=centre, size=size, **part)

    floor = -radius - dispenser["floor"]
    low, high = -radius + dispenser["station_wall_height"], -radius + dispenser["rail_wall_height"]
    for rail in range(dispenser["rails"]):
        box("floor", rail, (start, end), 0, (floor, -radius), gap / 2 + wall)
        box(
            "stop",
            rail,
            (start, -radius),
            dispenser["stop_offset"],
            (-radius, radius),
            dispenser["stop_width"] / 2,
        )
        for side, name in ((1, "left"), (-1, "right")):
            y = side * (gap + wall) / 2
            box(f"station_{name}", rail, (start, cut), y, (-radius, low), wall / 2)
            box(f"wall_{name}", rail, (cut, end), y, (-radius, high), wall / 2)
    # Its marker plate: a plate before the lowest station that carries the four markers, flat and facing up
    # (layout.dispenser_marker_centers), and reaches under the rails.
    markers = dispenser["markers"]
    centres = np.array(list(connect4_layout.dispenser_marker_centers(layout).values()))
    low, high = centres.min(axis=0) - markers["plate"] / 2, centres.max(axis=0) + markers["plate"] / 2
    high[1] = gap / 2 + wall  # up to the rails' far wall
    body.add_geom(
        name="dispenser_marker_plate",
        type=BOX,
        size=[*(high - low)[:2] / 2, 0.001],
        pos=[*(high + low)[:2] / 2, centres[0][2] - 0.0015],
        rgba=[0.85, 0.85, 0.9, 1],
        **solid,
    )
    for i, centre in connect4_layout.dispenser_marker_centers(layout).items():
        _add_marker(spec, folder, body, layout["markers"]["dictionary"], i, markers, centre, [1, 0, 0, 0])
    # What holds it all, drawn only: a leg under the marker plate, a post from the table to the top rail.
    top = connect4_layout.waiting(layout, dispenser["rails"] - 1, 2)
    table = table_z - T[2, 3]
    drawn = {"type": BOX, "rgba": [0.75, 0.75, 0.8, 1], "contype": 0, "conaffinity": 0}
    for name, x, y, z in (
        ("leg", (high + low)[0] / 2, (high + low)[1] / 2, centres[0][2]),
        ("post", top[0], 0, top[2]),
    ):
        body.add_geom(
            name=f"dispenser_{name}",
            size=[0.006, 0.006, (z - table) / 2],
            pos=[x, y, (z + table) / 2],
            **drawn,
        )


def _add_grid(spec, folder, layout, solid):
    """The grid, in its frame (so101_connect4.layout): contacts in hidden boxes, both faces drawn as a plate
    with round windows, the two posts with the markers, the depth guide."""
    grid, markers = layout["grid"], layout["markers"]
    pitch, row, gap, plate = grid["column_pitch"], grid["row_pitch"], grid["slot_gap"], grid["plate"]
    T = connect4_layout.grid_pose(layout)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, T[:3, :3].ravel())
    # A fixed body, not a mocap one: tokens at rest on a mocap body never fall asleep, they crept at 2 mm/s
    # and the tick slowed [MES-78]. The simulated person moves it by its pose in the model (move_grid, #260)
    body = spec.worldbody.add_body(name="grid", pos=T[:3, 3], quat=quat)
    x0, length = -pitch / 2, grid["columns"] * pitch
    xc = x0 + length / 2
    z0 = -row / 2  # the slots' floor
    height = grid["rows"] * row + grid["headroom"]  # up to the slot entries
    zc, entry = z0 + height / 2, z0 + height
    table = z0 - connect4_layout.slot_bottom(layout)
    hidden = {"type": BOX, "group": 3, **solid}
    drawn = {"type": BOX, "rgba": BLUE, "contype": 0, "conaffinity": 0}
    face_y = (gap + plate) / 2
    for side, name in ((1, "front"), (-1, "back")):
        body.add_geom(
            name=f"plate_{name}",
            size=[length / 2, plate / 2, height / 2],
            pos=[xc, side * face_y, zc],
            **hidden,
        )
    body.add_geom(
        name="slot_floor", size=[length / 2, gap / 2 + plate, 0.003], pos=[xc, 0, z0 - 0.003], **hidden
    )
    for i in range(grid["columns"] + 1):
        body.add_geom(
            name=f"wall_{i}",
            size=[grid["wall"] / 2, gap / 2, height / 2],
            pos=[x0 + i * pitch, 0, zc],
            **hidden,
        )
    # The depth guide: a V across the grid, open along it, which brings a token into the slot.
    out = (grid["depth_guide_opening"] - gap) / 2
    guide_h = grid["depth_guide_height"]
    for side, name in ((1, "front"), (-1, "back")):
        lean = -side * np.arctan2(out, guide_h) / 2  # half the angle about x: the strip's top leans outwards
        body.add_geom(
            name=f"guide_{name}",
            size=[length / 2, 0.001, np.hypot(out, guide_h) / 2],
            pos=[xc, side * (gap / 2 + out / 2 + 0.001), entry + guide_h / 2],
            quat=[np.cos(lean), np.sin(lean), 0, 0],
            **{**hidden, "group": 1, "rgba": [*BLUE[:3], 0.5]},
        )
    # What the camera sees: the two faces with their windows, the strip above them, the base, the posts.
    _window_plate(folder / "assets" / "connect4_plate.stl", grid)
    spec.add_mesh(name="connect4_plate", file="connect4_plate.stl")
    half_y = gap / 2 + plate
    for side in (1, -1):
        body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname="connect4_plate",
            pos=[0, side * face_y, 0],
            rgba=BLUE,
            contype=0,
            conaffinity=0,
            density=0,
        )
        body.add_geom(
            size=[length / 2, plate / 2, grid["headroom"] / 2],
            pos=[xc, side * face_y, entry - grid["headroom"] / 2],
            **drawn,
        )
    body.add_geom(
        name="grid_base", size=[length / 2, half_y, (z0 - table) / 2], pos=[xc, 0, (z0 + table) / 2], **drawn
    )
    margin = grid["frame_margin"]
    for name, x in (("near", x0 - margin / 2), ("far", x0 + length + margin / 2)):
        body.add_geom(
            name=f"post_{name}",
            size=[margin / 2, half_y, (entry - table) / 2],
            pos=[x, 0, (entry + table) / 2],
            **drawn,
        )
    face = np.zeros(
        4
    )  # a plane geom's x, y, z: the viewer's right (-x of the grid), up, the front face's normal
    mujoco.mju_mat2Quat(face, np.array([[-1.0, 0, 0], [0, 0, 1], [0, 1, 0]]).ravel())
    for i, center in connect4_layout.marker_centers(layout).items():
        _add_marker(spec, folder, body, markers["dictionary"], i, markers, center, face)


def _add_marker(spec, folder, body, dictionary, i, markers, center, quat):
    """A marker on its white plate, drawn only: a plane geom of `body` whose x is the marker image's right
    and y its up."""
    border = round(MARKER_PIXELS * (markers["plate"] - markers["size"]) / markers["size"] / 2)
    image = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)), i, MARKER_PIXELS
    )
    image = cv2.copyMakeBorder(image, *(border,) * 4, cv2.BORDER_CONSTANT, value=255)  # quiet zone
    cv2.imwrite(str(folder / f"aruco_{i}.png"), image)
    spec.add_texture(name=f"aruco_{i}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=f"aruco_{i}.png")
    material = spec.add_material(name=f"aruco_{i}")
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = f"aruco_{i}"
    body.add_geom(
        name=f"aruco_{i}",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[markers["plate"] / 2, markers["plate"] / 2, 0.001],
        pos=center,
        quat=quat,
        material=f"aruco_{i}",
        contype=0,
        conaffinity=0,
    )


def _window_plate(path, grid, segments=24):
    """A binary STL of one face: a tile per cell with a round window, in the grid frame, centred on y = 0."""
    pitch, row, h, r = grid["column_pitch"], grid["row_pitch"], grid["plate"] / 2, grid["window_diameter"] / 2
    corners = [np.arctan2(sz * row, sx * pitch) for sx in (1, -1) for sz in (1, -1)]
    angles = np.sort(np.r_[np.linspace(-np.pi, np.pi, segments, endpoint=False), corners])

    def edge(t):  # the tile's outline along the ray at angle t
        k = min(pitch / 2 / max(abs(np.cos(t)), 1e-9), row / 2 / max(abs(np.sin(t)), 1e-9))
        return k * np.cos(t), k * np.sin(t)

    triangles = []
    for column in range(grid["columns"]):
        for line in range(grid["rows"]):
            for t0, t1 in zip(angles, np.r_[angles[1:], angles[0]], strict=True):

                def at(p, y, cx=column * pitch, cz=line * row):
                    return [cx + p[0], y, cz + p[1]]

                o0, o1 = edge(t0), edge(t1)
                i0, i1 = (r * np.cos(t0), r * np.sin(t0)), (r * np.cos(t1), r * np.sin(t1))
                triangles += [(at(o0, h), at(i0, h), at(i1, h)), (at(o0, h), at(i1, h), at(o1, h))]  # front
                triangles += [
                    (at(o0, -h), at(i1, -h), at(i0, -h)),
                    (at(o0, -h), at(o1, -h), at(i1, -h)),
                ]  # back
                triangles += [
                    (at(i0, h), at(i0, -h), at(i1, -h)),
                    (at(i0, h), at(i1, -h), at(i1, h)),
                ]  # window
                triangles += [
                    (at(o0, h), at(o1, h), at(o1, -h)),
                    (at(o0, h), at(o1, -h), at(o0, -h)),
                ]  # closed
    triangles = np.array(triangles, np.float32)
    with open(path, "wb") as file:
        file.write(b"\0" * 80 + struct.pack("<I", len(triangles)))
        for triangle in triangles:
            file.write(struct.pack("<3f", 0, 0, 0) + triangle.tobytes() + b"\0\0")


class SimScene(Substitution):
    """The path of our scene, generated at launch: in sim, the moved camera, the markers with calibration, the
    lights of sim_scene.yaml's `lighting` entry if one is named, and the Connect Four game with connect4;
    empty (the provided scene) on the real arm."""

    def __init__(self, use_sim, calibration, lighting, connect4):
        super().__init__()
        self._use_sim, self._calibration, self._lighting = use_sim, calibration, lighting
        self._connect4 = connect4

    def describe(self):
        return "SimScene(use_sim, calibration, lighting, connect4)"

    def perform(self, context):
        if perform_substitutions(context, [self._use_sim]).lower() != "true":
            return ""
        config = Path(get_package_share_directory("so101_bringup")) / "config"
        scene = yaml.safe_load((config / "sim_scene.yaml").read_text())
        markers = None
        if perform_substitutions(context, [self._calibration]).lower() == "true":
            markers = yaml.safe_load((config / "markers_sim.yaml").read_text())
        lighting = None
        if name := perform_substitutions(context, [self._lighting]):
            if name not in scene["lighting"]:
                raise ValueError(f"lighting {name!r}: not in sim_scene.yaml ({', '.join(scene['lighting'])})")
            lighting = scene["lighting"][name]
        connect4 = None
        if perform_substitutions(context, [self._connect4]).lower() == "true":
            if markers:  # the calibration markers lie on the table, which the game lowers
                raise ValueError("calibration and connect4: one at a time")
            connect4 = connect4_layout.load()
        folder = tempfile.mkdtemp(prefix="so101_scene_")
        return str(
            generate(folder, camera=scene["camera"], markers=markers, lighting=lighting, connect4=connect4)
        )
