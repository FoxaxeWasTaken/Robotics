"""Prototype Connect Four scene (experiment only, never in the repo): MjSpec on a COPY of the provided scene.
Ground truth (the true grid pose) is used here only to score, as a test would (C-06)."""
import shutil, struct, tempfile
from pathlib import Path

import cv2, mujoco, numpy as np, yaml
from so101_sim import DEFAULT_SCENE

REPO = Path("/repo")
PAN_X = 0.0388
PX, TOKEN_D, TOKEN_T, GAP, WALL = 0.037, 0.034, 0.0065, 0.009, 0.0025
HOLE_R = 0.0145
TABLE_Z, TOP_H = -0.08, 0.25
COLS, ROWS = 7, 6
FLIP = np.diag([1.0, -1.0, -1.0])
P1_CAM = yaml.safe_load((REPO / "ros2_ws/src/so101_bringup/config/sim_scene.yaml").read_text())["camera"]
T_FILE = np.array(yaml.safe_load((REPO / "ros2_ws/src/so101_perception/config/camera_extrinsics.yaml").read_text())["T_base_cam"])
PARK = [1.9, -1.7, 1.5, 0.7, 0.0]
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
RED = (0.80, 0.05, 0.10, 1); YEL = (0.95, 0.78, 0.05, 1); BLUE = (0.06, 0.20, 0.72, 1)


def look_at(pos, target):
    f = np.asarray(target, float) - pos; f /= np.linalg.norm(f)
    x = np.cross(f, [0, 0, 1.0]); x /= np.linalg.norm(x)
    return x, np.cross(x, f)


class Grid:
    """Frame of P2-02 §6, right-handed: origin = center of cell (col 0, row 0) in the mid-plane, x radial outward
    (col 0 nearest the arm), y = z x x (towards the camera for az < 0), z up."""
    def __init__(s, az=-51.0, pz=0.034, r0=0.14, margin=0.045, marker=0.03, quiet=1.3, table=TABLE_Z, top=TOP_H):
        s.az, s.pz, s.r0, s.m, s.marker, s.plate, s.table, s.top = az, pz, r0, margin, marker, marker * quiet, table, top
        a = np.radians(az)
        s.x = np.array([np.cos(a), np.sin(a), 0.0]); s.z = np.array([0, 0, 1.0]); s.y = np.cross(s.z, s.x)
        s.R = np.column_stack([s.x, s.y, s.z])
        s.bottom = top - ROWS * pz          # slot bottom above the table
        s.t = np.array([PAN_X + r0 * s.x[0], r0 * s.x[1], table + s.bottom + TOKEN_D / 2])
        s.T = np.eye(4); s.T[:3, :3] = s.R; s.T[:3, 3] = s.t

    def cell(s, c, r, y=0.0):  # in the grid frame
        return np.array([c * PX, y, r * s.pz])

    def markers(s):
        """id -> center in the grid frame, on the front face, 4 frame corners. Right of the image = -x."""
        y = GAP / 2 + WALL + 0.0004
        xl, xr = -PX / 2 - s.m / 2, 6.5 * PX + s.m / 2
        zb, zt = -s.pz / 2 - s.bottom + s.plate / 2 + 0.002, s.top - s.bottom - s.pz / 2 - s.plate / 2 - 0.002
        return {10: (xl, y, zt), 11: (xr, y, zt), 12: (xl, y, zb), 13: (xr, y, zb)}

    def marker_corners(s, i):
        """cv2.aruco order (TL, TR, BR, BL as seen from the front): viewer's right = -x, up = +z."""
        c = np.array(s.markers()[i]); h = s.marker / 2
        r, u = np.array([-1.0, 0, 0]), np.array([0, 0, 1.0])
        return np.array([c - r * h + u * h, c + r * h + u * h, c + r * h - u * h, c - r * h - u * h])


def stl_panel(path, g, n=24):
    """One perforated panel, thickness along y, centered on y = 0: 42 tiles + 4 border boxes are separate geoms."""
    tris = []
    h = WALL / 2
    corner = [np.arctan2(sz * g.pz / 2, sx * PX / 2) for sx in (1, -1) for sz in (1, -1)]
    th = np.sort(np.r_[np.linspace(-np.pi, np.pi, n, endpoint=False), corner])
    def outer(t):
        k = min(PX / 2 / max(abs(np.cos(t)), 1e-9), g.pz / 2 / max(abs(np.sin(t)), 1e-9))
        return k * np.cos(t), k * np.sin(t)
    for c in range(COLS):
        for r in range(ROWS):
            cx, cz = c * PX, r * g.pz
            for t0, t1 in zip(th, np.r_[th[1:], th[0]]):
                o0, o1 = outer(t0), outer(t1)
                i0 = (HOLE_R * np.cos(t0), HOLE_R * np.sin(t0)); i1 = (HOLE_R * np.cos(t1), HOLE_R * np.sin(t1))
                P = lambda p, y: np.array([cx + p[0], y, cz + p[1]])
                # front (+y), back (-y), hole wall
                tris += [(P(o0, h), P(i0, h), P(i1, h)), (P(o0, h), P(i1, h), P(o1, h))]
                tris += [(P(o0, -h), P(i1, -h), P(i0, -h)), (P(o0, -h), P(o1, -h), P(i1, -h))]
                tris += [(P(i0, h), P(i0, -h), P(i1, -h)), (P(i0, h), P(i1, -h), P(i1, h))]
                # tile side walls, so that the mesh is closed (they are inside the panel but harmless)
                tris += [(P(o0, h), P(o1, h), P(o1, -h)), (P(o0, h), P(o1, -h), P(o0, -h))]
    tris = np.array(tris, np.float32)
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", 0, 0, 0) + t.tobytes() + b"\0\0")
    return len(tris)


def mat2quat(M):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(M, float).ravel()); return q


def generate(g: Grid, cam=None, table="lowered", lighting=None, tmp=None, shadow=None, reflect=None, static_board=None):
    """table='lowered': floor at -8 cm, world = base_link kept. 'raised': base +8 cm, floor at 0, world != base_link."""
    folder = Path(tmp or tempfile.mkdtemp(prefix="c4_")) / "so101"
    shutil.copytree(Path(DEFAULT_SCENE).parent, folder)
    spec = mujoco.MjSpec.from_file(str(folder / Path(DEFAULT_SCENE).name))
    rise = 0.0
    if shadow is not None:
        spec.visual.quality.shadowsize = shadow
    if reflect is not None:
        spec.material("groundplane").reflectance = reflect
    if table == "lowered":
        spec.geom("floor").pos = [0, 0, TABLE_Z]
    else:
        spec.body("base").pos = [0, 0, -TABLE_Z]; rise = -TABLE_Z
    spec.body("drop_box").pos = [0.05, -1.5, TABLE_Z + rise]  # out of the play area
    if cam:
        c = spec.camera("external_cam"); c.pos = cam["pos"]
        c.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES; c.alt.xyaxes = cam["xyaxes"]
    wood = spec.add_material(name="riser"); wood.rgba = [0.55, 0.45, 0.33, 1]
    spec.worldbody.add_geom(name="riser", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.075, 0.07, 0.04],
                            pos=[-0.01, 0, TABLE_Z + rise + 0.04], material="riser")
    blue = spec.add_material(name="grid_blue"); blue.rgba = BLUE
    red = spec.add_material(name="tok_red"); red.rgba = RED
    yel = spec.add_material(name="tok_yel"); yel.rgba = YEL
    stl_panel(folder / "assets" / "grid_panel.stl", g)
    spec.add_mesh(name="grid_panel", file="grid_panel.stl")
    b = spec.worldbody.add_body(name="grid", pos=g.t + rise * g.z, quat=mat2quat(g.R), mocap=True)
    mesh = dict(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="grid_panel", material="grid_blue", contype=0, conaffinity=0, density=0)
    b.add_geom(pos=[0, GAP / 2 + WALL / 2, 0], **mesh)
    b.add_geom(pos=[0, -GAP / 2 - WALL / 2, 0], **mesh)
    bx = dict(type=mujoco.mjtGeom.mjGEOM_BOX, material="grid_blue", contype=0, conaffinity=0, density=0)
    w = g.m / 2; cz = (ROWS - 1) * g.pz / 2; hz = ROWS * g.pz / 2; cx = (COLS - 1) * PX / 2; hx = COLS * PX / 2
    Y = GAP / 2 + WALL
    zb = -g.bottom - g.pz / 2  # table, in the grid frame
    for y0 in (0.0,):  # full-thickness frame: side posts, bottom rail, slot walls
        b.add_geom(pos=[-PX / 2 - w, y0, (zb + hz * 2 - g.pz / 2) / 2], size=[w, Y, (hz * 2 - zb - g.pz / 2) / 2], **bx)
        b.add_geom(pos=[6.5 * PX + w, y0, (zb + hz * 2 - g.pz / 2) / 2], size=[w, Y, (hz * 2 - zb - g.pz / 2) / 2], **bx)
        b.add_geom(pos=[cx, y0, (zb - g.pz / 2) / 2], size=[hx, Y, (-g.pz / 2 - zb) / 2], **bx)
    for c in range(1, COLS):
        b.add_geom(pos=[(c - 0.5) * PX, 0, cz], size=[0.001, GAP / 2, hz], **bx)
    white = spec.add_material(name="plate_white"); white.rgba = [1, 1, 1, 1]
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for i, c in g.markers().items():
        px = 600; border = round(px * (g.plate - g.marker) / g.marker / 2)
        img = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(dictionary, i, px), *(border,) * 4, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(str(folder / f"aruco_{i}.png"), img)
        spec.add_texture(name=f"aruco_{i}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=f"aruco_{i}.png")
        m = spec.add_material(name=f"aruco_{i}"); m.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = f"aruco_{i}"
        # plane normal = +y_grid; local x = viewer's right = -x_grid, local y = viewer's up = +z_grid
        b.add_geom(name=f"aruco_{i}", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[g.plate / 2, g.plate / 2, 0.001], pos=c,
                   quat=mat2quat(np.column_stack([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])), material=f"aruco_{i}", contype=0, conaffinity=0)
    if static_board is not None:  # timing runs in the real driver: a full board, no free joints
        q = mat2quat(g.R @ np.column_stack([[1, 0, 0], [0, 0, -1], [0, 1, 0]]))
        for r in range(ROWS):
            for c in range(COLS):
                spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[TOKEN_D / 2, TOKEN_T / 2, 0], pos=g.T[:3, :3] @ g.cell(c, r) + g.t,
                                        quat=q, material="tok_red" if static_board[r, c] == 1 else "tok_yel", contype=0, conaffinity=0)
    for name, mat, n in (("r", "tok_red", 21), ("y", "tok_yel", 22)) if static_board is None else ():  # y21: the one in the jaws
        for i in range(n):
            t = spec.worldbody.add_body(name=f"tok_{name}{i}", pos=[-0.2 + i * 0.02, 0.1 if name == "r" else -0.1, -0.3])
            t.add_freejoint(name=f"tokj_{name}{i}")
            t.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[TOKEN_D / 2, TOKEN_T / 2, 0], material=mat, contype=0, conaffinity=0, mass=0.003)
    if lighting:
        factor = lighting["scale"] * np.asarray(lighting["tint"], float)
        for light in [*spec.lights, spec.visual.headlight]:
            light.diffuse = np.asarray(light.diffuse) * factor
            light.ambient = np.asarray(light.ambient) * factor
            light.specular = np.asarray(light.specular) * factor
    path = folder / "scene_so101.xml"
    path.write_text(spec.to_xml())
    return path


class World:
    def __init__(s, g, W=320, H=240, **kw):
        s.g, s.W, s.H, s.rise = g, W, H, (-TABLE_Z if kw.get("table") == "raised" else 0.0)
        s.path = generate(g, **kw)
        s.m = mujoco.MjModel.from_xml_path(str(s.path)); s.d = mujoco.MjData(s.m)
        s.r = mujoco.Renderer(s.m, height=H, width=W)
        s.cam = s.m.camera("external_cam").id
        f = H / (2 * np.tan(np.radians(s.m.cam_fovy[s.cam]) / 2))
        s.K = np.array([[f, 0, (W - 1) / 2], [0, f, (H - 1) / 2], [0, 0, 1.0]])  # opencv_intrinsics
        adr = s.m.joint("pickup_object_free").qposadr[0]
        s.d.qpos[adr:adr + 3] = [-1.0, -1.0, TABLE_Z + 0.03]
        s.arm(PARK); s.tok_adr = {n: s.m.joint(f"tokj_{n}").qposadr[0] for c, k in (("r", 21), ("y", 22)) for n in [f"{c}{i}" for i in range(k)]}
        s.tok_home = {n: s.d.qpos[a:a + 7].copy() for n, a in s.tok_adr.items()}
        root = s.m.body("base").id
        s.arm_geoms = {i for i in range(s.m.ngeom) if s.m.body_rootid[s.m.geom_bodyid[i]] == root}
        mujoco.mj_forward(s.m, s.d)

    def arm(s, q, gripper=None):
        for n, v in zip(JOINTS, q):
            s.d.qpos[s.m.joint(n).qposadr[0]] = v
        lo, hi = s.m.joint("gripper").range
        s.d.qpos[s.m.joint("gripper").qposadr[0]] = hi if gripper is None else gripper

    def set_grid(s, T):
        """Moves the grid body and returns; tokens are placed against it by board()."""
        s.d.mocap_pos[0] = T[:3, 3] + s.rise * s.g.z; s.d.mocap_quat[0] = mat2quat(T[:3, :3])

    def board(s, cells, T=None, rng=None, pz_stack=None):
        """cells[r, c] in {0, 1 human red, 2 robot yellow}: tokens placed at rest with their real play."""
        T = s.g.T if T is None else T
        for n, a in s.tok_adr.items():
            s.d.qpos[a:a + 7] = s.tok_home[n]
        idx = {1: 0, 2: 0}
        q = mat2quat(T[:3, :3] @ np.column_stack([[1, 0, 0], [0, 0, -1], [0, 1, 0]]))
        pz = s.g.pz if pz_stack is None else pz_stack
        for r in range(ROWS):
            for c in range(COLS):
                v = cells[r, c]
                if not v:
                    continue
                n = f"{'r' if v == 1 else 'y'}{idx[v]}"; idx[v] += 1
                j = np.zeros(3) if rng is None else rng.uniform([-0.0005, -0.00125, 0], [0.0005, 0.00125, 0])
                p = T[:3, :3] @ (np.array([c * PX, 0, r * pz]) + j) + T[:3, 3] + s.rise * s.g.z
                a = s.tok_adr[n]; s.d.qpos[a:a + 3] = p; s.d.qpos[a + 3:a + 7] = q

    def render(s):
        mujoco.mj_forward(s.m, s.d)
        s.r.update_scene(s.d, camera="external_cam")
        return s.r.render().copy()

    def seg(s):
        mujoco.mj_forward(s.m, s.d)
        s.r.enable_segmentation_rendering(); s.r.update_scene(s.d, camera="external_cam")
        out = s.r.render().copy(); s.r.disable_segmentation_rendering()
        return out[..., 0]

    def T_true(s):
        """The camera actually rendered, OpenCV optical frame, in base_link (test only)."""
        mujoco.mj_forward(s.m, s.d)
        T = np.eye(4); T[:3, :3] = s.d.cam_xmat[s.cam].reshape(3, 3) @ FLIP; T[:3, 3] = s.d.cam_xpos[s.cam] - [0, 0, s.rise]
        return T

    def close(s):
        s.r.close(); shutil.rmtree(s.path.parent.parent, ignore_errors=True)


def project(pts, T_base_x, T_base_cam, K):
    """Points of frame x -> pixels."""
    pts = np.atleast_2d(pts)
    p = (np.linalg.inv(T_base_cam) @ T_base_x @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
    uv = (K @ p.T).T
    return uv[:, :2] / uv[:, 2:], p[:, 2]


def detect(rgb):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    params = cv2.aruco.DetectorParameters(); params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
    corners, ids, _ = det.detectMarkers(gray)
    return {} if ids is None else {int(i): c[0] for i, c in zip(ids.ravel(), corners)}


def pnp(found, g, K):
    """T_cam_grid from the corners of the grid's markers, phase-1 solver (SQPNP + LM, calibration.py)."""
    ids = [i for i in sorted(found) if i in g.markers()]
    if len(ids) < 2:
        return None
    w = np.vstack([g.marker_corners(i) for i in ids]); im = np.vstack([found[i] for i in ids]).astype(float)
    _, rv, tv = cv2.solvePnP(w, im, K, None, flags=cv2.SOLVEPNP_SQPNP)
    rv, tv = cv2.solvePnPRefineLM(w, im, K, None, rv, tv)
    T = np.eye(4); T[:3, :3] = cv2.Rodrigues(rv)[0]; T[:3, 3] = tv.ravel()
    err = np.linalg.norm(cv2.projectPoints(w, rv, tv, K, None)[0][:, 0] - im, axis=1).mean()
    return T, err, ids


def random_board(rng, moves):
    cells = np.zeros((ROWS, COLS), int); h = [0] * COLS; p = 1
    for _ in range(moves):
        c = rng.choice([i for i in range(COLS) if h[i] < ROWS])
        cells[h[c], c] = p; h[c] += 1; p = 3 - p
    return cells


def disc_masks(uv, radius, shape):
    return [cv2.circle(np.zeros(shape[:2], np.uint8), (int(round(a)), int(round(b))), int(round(radius)), 255, -1) > 0 for a, b in uv]
