"""Connect Four sim scene prototype and measurements. Throwaway spike, input of #76 (P2.1b) and #79.

Scene generated with mujoco.MjSpec from a copy of the provided scene (never sim/), as so101_bringup.sim_scene.
Dimensions are the assumed model of P2-02 §1, all in PARAMS; GAME_OPTIONS is the measured working set.
reach.py first (brute-force reach map, reach.npy, 3.7 MB: never commit it), the IK of `move` seeds from it.

  ./run.sh scene OUT        the scene and two renders
  ./run.sh cost / cost2     (a) tick cost per scene and per contact setting
  ./run.sh settle           (a) tokens asleep and tick cost over a game of drops
  ./run.sh drop             (b) 42 tokens per game: cells, jams, settle time
  ./run.sh grip             (c) token between the jaws, roll +-90 deg, moves at 0.5-1.5 rad/s
  ./run.sh move [--n N --offset M --blind --set riser=0.1,gap=0.0085]  full robot moves: handover to cell
  ./run.sh capture          TCP error sweep on one column
  ./run.sh repark           (d) every token back to its parking place
  ./run.sh riser            question 1: raise the base or lower the floor

#76, one layout (P2-02 §7), the brain's Kinematics, so through tools/so101 run (the workspace built):
  tools/so101 run 'python3 -u /repo/docs/project/experiences/connect4/scene/exp_connect4_scene.py spike'
      clearance on the 7 columns, 100 drops, 100 with +-5 mm on the TCP, capture sweep: go / no-go
  ... exp_connect4_scene.py stacks [--n N]   N grids filled through the guide, read back with reading/

drv.py: the real DriverNode on this scene (tools/so101 run --out <here> 'python3 -u /out/drv.py').
render_cost.py: OSMesa frame cost per quality setting.
"""

import argparse
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path

import mujoco
import numpy as np

ASSETS = Path("/opt/so101/sim/so101_sim/assets/so101")
SCENE = "scene_tek5.xml"
PAN_X = 0.0388  # m: shoulder_pan axis in base_link (04 §1.1)
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
STEPS_PER_TICK = 10  # SO101Sim: send_action and get_observation, 5 mj_step each, 2 ms: 20 ms per tick (MES-02)

PARAMS = {
    "pitch": 0.037,  # column pitch, along the grid
    "gap": 0.0095,  # slot gap, across the grid (8.5-10.5 mm)
    "d": 0.034,  # token diameter
    "t": 0.0065,  # token thickness
    "wall": 0.0022,  # column wall, along the grid: leaves pitch - wall - d of play
    "plate": 0.003,  # front and back plates
    "entry": 0.25,  # slot entries above the table
    "riser": 0.08,
    "r0": 0.14,  # first column from the pan axis; the last is at r0 + 6 pitch = 0.362
    "az": 0.0,  # azimuth of the grid plane around the pan axis, rad
    "plane": 0.0,  # m, the grid plane off the pan axis along its normal (-sin az, cos az, 0) (#76)
    "rows": 6,
    "cols": 7,
    "tokens": 42,
    "guide_h": 0.02,  # depth guide height above the entry
    "guide_open": 0.02,  # its opening at the top
    "mass": 0.004,  # token mass, kg
    "friction": 0.3,  # token sliding friction
    "solref": 0.004,  # token contact time constant, s (default 0.02)
    "margin": 0.0,
    "condim": 3,
    "park": "floor",  # floor: spread on the floor outside the camera view; sleep: same, sleeping allowed
    "timestep": 0.002,
    "lattice": True,  # visual bars of the grid faces (no contact)
    "fast_render": True,
    "camera": {"pos": [0.29, -0.52, 0.12], "xyaxes": [1, 0, 0, 0, 0.1, 0.995]},
    "solmix": 1.0,  # token solmix: large = the token's solref rules its contacts with the jaws (default 1)
    "cone": "pyramidal",
    "impratio": 1.0,
    "noslip": 0,
    "sleep_tol": 1e-4,  # m/s-like threshold of MuJoCo's sleeping islands (its default)
    "inertia_scale": 1.0,  # token rotational inertia, sim-only: light discs in stiff contacts buzz
    "multiccd": True,
    "iterations": 100,  # solver iterations (provided default)
    "tolerance": 1e-8,  # solver tolerance (provided default)
    "damping": 0.0,  # free-joint damping, linear N s/m (the angular one is scaled by r^2)
}


def stack_z(p, row):
    """Height above the table of a token center in `row`: tokens rest on each other, not at the column pitch."""
    return bottom_z(p) + p["d"] * (row + 0.5)


def bottom_z(p):
    return p["entry"] - p["rows"] * p["d"] - 0.002


def park_xy(i):
    """Spread on the floor behind the robot, 5 cm apart: out of every view, no token touches another."""
    return np.array([-0.5 - 0.05 * (i % 7), -0.3 + 0.05 * (i // 7)])


def add_grid(spec, p, table_z):
    """Static grid: collisions in hidden boxes (group 3), its faces as visual bars so tokens stay visible."""
    r = p["r0"] + p["pitch"] * np.arange(p["cols"])
    x0, x1 = r[0] - p["pitch"] / 2, r[-1] + p["pitch"] / 2
    L, zb, H = x1 - x0, bottom_z(p), p["entry"] - bottom_z(p)
    xc, zc = (x0 + x1) / 2, zb + H / 2
    origin = np.array([PAN_X, 0, table_z]) + p["plane"] * np.array([-np.sin(p["az"]), np.cos(p["az"]), 0])
    grid = spec.worldbody.add_body(
        name="grid", pos=origin, quat=[np.cos(p["az"] / 2), 0, 0, np.sin(p["az"] / 2)]
    )
    wall = {"type": mujoco.mjtGeom.mjGEOM_BOX, "group": 3, "friction": [p["friction"], 0.005, 0.0001]}
    wall |= {"solref": [p["solref"], 1], "margin": p["margin"], "condim": p["condim"]}
    hy = (p["gap"] + p["plate"]) / 2
    for side, name in ((1, "front"), (-1, "back")):
        grid.add_geom(name=f"plate_{name}", size=[L / 2, p["plate"] / 2, H / 2], pos=[xc, side * hy, zc], **wall)
    grid.add_geom(name="grid_bottom", size=[L / 2, p["gap"] / 2 + p["plate"], 0.003], pos=[xc, 0, zb - 0.003], **wall)
    for i in range(p["cols"] + 1):
        x = x0 + i * p["pitch"]
        grid.add_geom(name=f"wall_{i}", size=[p["wall"] / 2, p["gap"] / 2, H / 2], pos=[x, 0, zc], **wall)
    # Depth guide: a V across the grid, open along it.
    out = (p["guide_open"] - p["gap"]) / 2
    half = np.hypot(out, p["guide_h"]) / 2
    tilt = np.arctan2(out, p["guide_h"])
    for side, name in ((1, "front"), (-1, "back")):
        a = -side * tilt / 2  # about +x: the strip's top leans outwards
        grid.add_geom(
            name=f"guide_{name}",
            size=[L / 2, 0.001, half],
            pos=[xc, side * (p["gap"] / 2 + out / 2 + 0.001), p["entry"] + p["guide_h"] / 2],
            quat=[np.cos(a), np.sin(a), 0, 0],
            rgba=[0.1, 0.2, 0.7, 0.5],
            **{**wall, "group": 1},
        )
    # Feet, so the grid stands on the table; they touch nothing that moves.
    grid.add_geom(
        name="grid_foot",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[L / 2 + 0.01, 0.05, zb / 2 - 0.003],
        pos=[xc, 0, zb / 2 - 0.003],
        rgba=[0.08, 0.15, 0.6, 1],
        contype=0,
        conaffinity=0,
    )
    if p["lattice"]:
        bar = dict(type=mujoco.mjtGeom.mjGEOM_BOX, rgba=[0.08, 0.15, 0.6, 1], contype=0, conaffinity=0, group=2)
        for side in (1, -1):
            for i in range(p["cols"] + 1):
                x = x0 + i * p["pitch"]
                grid.add_geom(size=[0.0015, p["plate"] / 2, H / 2], pos=[x, side * hy, zc], **bar)
            for j in range(p["rows"] + 1):
                z = zb + j * p["d"]
                grid.add_geom(size=[L / 2, p["plate"] / 2, 0.0015], pos=[xc, side * hy, z], **bar)


def add_tokens(spec, p, table_z):
    for i in range(p["tokens"]):
        x, y = park_xy(i)
        body = spec.worldbody.add_body(name=f"token_{i}", pos=[x, y, table_z + p["t"] / 2])
        if p["inertia_scale"] != 1.0:
            r, t, k = p["d"] / 2, p["t"], p["inertia_scale"]
            body.explicitinertial = True
            body.mass = p["mass"]
            side = p["mass"] * (3 * r * r + t * t) / 12
            body.inertia = [k * side, k * side, k * p["mass"] * r * r / 2]
        if p["damping"]:
            joint = body.add_joint(name=f"token_{i}_free", type=mujoco.mjtJoint.mjJNT_FREE)
            joint.damping = [p["damping"], 0, 0]
        else:
            body.add_freejoint(name=f"token_{i}_free")
        body.add_geom(
            name=f"token_{i}",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[p["d"] / 2, p["t"] / 2, 0],
            mass=p["mass"] if p["inertia_scale"] == 1.0 else 0.0,
            rgba=[0.9, 0.1, 0.1, 1] if i % 2 == 0 else [0.95, 0.8, 0.05, 1],
            friction=[p["friction"], 0.005, 0.0001],
            solref=[p["solref"], 1],
            margin=p["margin"],
            condim=p["condim"],
            solmix=p["solmix"],
        )


def build(folder, riser_mode="lower", **over):
    """The game scene in folder; riser_mode: lower (the base stays at the origin) or raise (ADR-036 update)."""
    p = PARAMS | over
    folder = Path(folder) / "so101"
    shutil.copytree(ASSETS, folder, dirs_exist_ok=True)
    spec = mujoco.MjSpec.from_file(str(folder / SCENE))
    spec.option.timestep = p["timestep"]
    spec.option.impratio = p["impratio"]
    spec.option.noslip_iterations = p["noslip"]
    spec.option.iterations = p["iterations"]
    spec.option.tolerance = p["tolerance"]
    if not p["multiccd"]:
        spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_MULTICCD
    if p["cone"] == "elliptic":
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    table_z = 0.0
    if riser_mode == "lower":
        table_z = -p["riser"]
        spec.geom("floor").pos[2] = table_z
    elif riser_mode == "raise":
        spec.body("base").pos[2] += p["riser"]
    spec.worldbody.add_geom(
        name="riser",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.07, 0.07, p["riser"] / 2],
        pos=[0.01, 0, table_z + p["riser"] / 2],
        rgba=[0.3, 0.3, 0.3, 1],
    )
    # The phase-1 objects stay: connect() needs pickup_object_free. Ghosts, so nothing has to park them.
    spec.body("drop_box").pos = [2.0, 0.0, table_z]
    for geom in spec.body("pickup_object").geoms:
        geom.contype = geom.conaffinity = 0
    spec.body("pickup_object").gravcomp = 1.0
    spec.delete(spec.geom("pickup_object_visual"))
    add_grid(spec, p, table_z)
    add_tokens(spec, p, table_z)
    if p["camera"]:  # external_cam moved in front of the grid face (one camera, moved by the scene generator)
        cam = spec.camera("external_cam")
        cam.pos = p["camera"]["pos"]
        cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
        cam.alt.xyaxes = p["camera"]["xyaxes"]
    if p["fast_render"]:  # OSMesa: 183 ms a frame as built, 27 ms like this [render_cost.py]
        spec.visual.quality.shadowsize = 0
        spec.material("groundplane").reflectance = 0
    if p["park"] == "sleep":
        spec.option.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP
        spec.option.sleep_tolerance = p["sleep_tol"]
    path = folder / "scene_connect4.xml"
    path.write_text(spec.to_xml())
    return path, p, table_z


def provided():
    folder = Path(tempfile.mkdtemp(prefix="so101_p1_")) / "so101"
    shutil.copytree(ASSETS, folder)
    return folder / SCENE


class Sim:
    """What the driver does to MuJoCo: 10 steps per tick, ctrl written before them, no ROS."""

    def __init__(self, path, p=None, table_z=0.0):
        self.m = mujoco.MjModel.from_xml_path(str(path))
        self.d = mujoco.MjData(self.m)
        self.p, self.table_z = p, table_z
        self.jq = {n: self.m.joint(n).qposadr[0] for n in ARM}
        self.act = {n: self.m.actuator(n).id for n in ARM}
        self.home()
        mujoco.mj_forward(self.m, self.d)

    def home(self, q=(0.0, -1.2, 1.2, 0.3, 0.0, 1.0)):
        """A bent pose far from the grid."""
        for n, v in zip(ARM, q, strict=True):
            self.d.qpos[self.jq[n]] = v
            self.d.ctrl[self.act[n]] = v

    def token_adr(self, i):
        j = self.m.joint(f"token_{i}_free")
        return j.qposadr[0], j.dofadr[0]

    def column_xy(self, col):
        """The column's centre in the grid plane, seen from above."""
        r, a = self.p["r0"] + self.p["pitch"] * col, self.p["az"]
        return np.array([PAN_X, 0]) + r * np.array([np.cos(a), np.sin(a)]) + self.p["plane"] * np.array([-np.sin(a), np.cos(a)])

    def place(self, i, col, z_above_table, yaw=0.0, dy=0.0, dx=0.0):
        """Teleport token i, its disc in the grid plane (axis along the grid normal)."""
        q, v = self.token_adr(i)
        a = self.p["az"]
        xy = self.column_xy(col) + dx * np.array([np.cos(a), np.sin(a)]) + dy * np.array([-np.sin(a), np.cos(a)])
        self.d.qpos[q : q + 3] = [xy[0], xy[1], self.table_z + z_above_table]
        # Rotate the cylinder axis (z) onto the grid normal, then yaw + az.
        q1 = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0])
        q2 = np.array([np.cos(a / 2), 0, 0, np.sin(a / 2)])
        out = np.zeros(4)
        mujoco.mju_mulQuat(out, q2, q1)
        q3 = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        final = np.zeros(4)
        mujoco.mju_mulQuat(final, out, q3)
        self.d.qpos[q + 3 : q + 7] = final
        self.d.qvel[v : v + 6] = 0

    def fill(self, tokens_per_col, settle=0.3):
        """Tokens at their rest cells without falling: tokens_per_col[c] tokens in column c."""
        i = 0
        for c, n in enumerate(tokens_per_col):
            for row in range(n):
                self.place(i, c, stack_z(self.p, row) + 0.0003)
                i += 1
        self.step(int(settle / self.m.opt.timestep))
        return i

    def step(self, n):
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)

    def tick(self):
        self.step(STEPS_PER_TICK)

    def token_state(self, i):
        q, v = self.token_adr(i)
        return self.d.qpos[q : q + 7].copy(), self.d.qvel[v : v + 6].copy()

    def cell_of(self, i):
        """(col, row) of token i from its pose, None if it lies in no cell; ground truth, spike only."""
        pos, p = self.token_state(i)[0][:3], self.p
        a = p["az"]
        rel = pos[:2] - np.array([PAN_X, 0])
        along = rel @ np.array([np.cos(a), np.sin(a)])
        across = rel @ np.array([-np.sin(a), np.cos(a)]) - p["plane"]
        z = pos[2] - self.table_z
        col = (along - p["r0"]) / p["pitch"]
        row = (z - bottom_z(p)) / p["d"] - 0.5
        if abs(across) > p["gap"] / 2 or abs(col - round(col)) > 0.25 or abs(row - round(row)) > 0.25:
            return None
        return int(round(col)), int(round(row))


# ------------------------------------------------------------------------------------------------- (a)
def arm_motion(sim, t):
    """A 1 rad/s-ish sweep of pan, lift and elbow, away from the grid: the arm's contacts stay as in phase 1."""
    sim.d.ctrl[sim.act["shoulder_pan"]] = 1.2 * np.sin(t) - 0.6
    sim.d.ctrl[sim.act["wrist_flex"]] = 0.3 + 0.4 * np.sin(1.3 * t)


def bench(sim, ticks=500, move=True, on_tick=None):
    """Wall time of 10 steps at the driver's rhythm, without pacing, one thread."""
    durations, ncon, nefc = [], [], []
    for k in range(ticks):
        if move:
            arm_motion(sim, k * 0.02)
        if on_tick:
            on_tick(k)
        t0 = time.perf_counter()
        sim.tick()
        durations.append(time.perf_counter() - t0)
        ncon.append(sim.d.ncon)
        nefc.append(sim.d.nefc)
    us = np.array(durations) * 1e3
    return {
        "tick_ms_p50": round(float(np.median(us)), 3),
        "tick_ms_p99": round(float(np.percentile(us, 99)), 3),
        "tick_ms_max": round(float(us.max()), 3),
        "rtf": round(0.02 / float(np.mean(durations)), 1),
        "ncon": int(np.median(ncon)),
        "ncon_max": int(max(ncon)),
        "nefc": int(np.median(nefc)),
    }


def cost(args):
    results = {}
    sim = Sim(provided())
    results["phase-1 scene"] = bench(sim)
    print("phase-1 scene", results["phase-1 scene"], flush=True)
    board21 = [3, 3, 3, 3, 3, 3, 3]
    board42 = [6] * 7
    for name, over in (("cylinder, default margin", {}), ("sleep enabled", {"park": "sleep"})):
        for label, cols in (("empty board", [0] * 7), ("21 in grid", board21), ("42 in grid", board42)):
            path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
            sim = Sim(path, p, tz)
            sim.fill(cols)
            r = bench(sim)
            results[f"{name} / {label}"] = r
            print(f"{name} / {label}", r, f"nv={sim.m.nv} ngeom={sim.m.ngeom}", flush=True)
    return results


def scene(args):
    import cv2

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    path, p, tz = build(out)
    print(path)
    sim = Sim(path, p, tz)
    print("nq", sim.m.nq, "nv", sim.m.nv, "ngeom", sim.m.ngeom, "nbody", sim.m.nbody, "extent", sim.m.stat.extent)
    sim.fill([2, 3, 6, 4, 1, 0, 5])
    sim.step(100)
    sim.m.vis.global_.offwidth = 640
    r = mujoco.Renderer(sim.m, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [PAN_X + 0.25, 0, 0.05]
    cam.distance, cam.azimuth, cam.elevation = 0.8, 90, -15
    r.update_scene(sim.d, camera=cam)
    cv2.imwrite(str(out / "front.png"), r.render()[:, :, ::-1])
    cam.azimuth = 35
    r.update_scene(sim.d, camera=cam)
    cv2.imwrite(str(out / "persp.png"), r.render()[:, :, ::-1])
    r.close()


# ------------------------------------------------------------------------------------------------- (b)
def drop_height(p):
    """Center of a token released by drop_token, in the grid plane: 5 mm above the guide."""
    return p["entry"] + p["guide_h"] + p["d"] / 2 + 0.005


def wait_settled(sim, i, timeout=3.0, window=0.1, tol=0.0003, ticks=None):
    """Sim time until token i stayed within `tol` for `window`; None on timeout. Speed is no criterion: a settled
    stack rattles at 20 mm/s and 0.05 mm."""
    dt = sim.m.opt.timestep * STEPS_PER_TICK
    n = int(window / dt)
    hist = []
    t = 0.0
    while t < timeout:
        t0 = time.perf_counter()
        sim.step(int(round(0.02 / sim.m.opt.timestep)))
        if ticks is not None:
            ticks.append(time.perf_counter() - t0)
        t += 0.02
        hist.append(sim.token_state(i)[0][:3].copy())
        if len(hist) >= n and np.ptp(np.array(hist[-n:]), axis=0).max() < tol:
            return t - window
    return None


def tilt_deg(sim, i):
    """Angle between the token axis and the grid normal."""
    q = sim.token_state(i)[0][3:7]
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, q)
    a = sim.p["az"]
    return float(np.degrees(np.arccos(min(1.0, abs(mat.reshape(3, 3)[:, 2] @ [-np.sin(a), np.cos(a), 0])))))


def one_game(over, seed, offset=0.0):
    """42 tokens dropped one by one in a random order; returns the failures and the settle times."""
    path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
    sim = Sim(path, p, tz)
    sim.home((-1.5, -1.2, 1.2, 0.3, 0.0, 1.0))
    sim.step(100)
    rng = np.random.default_rng(seed)
    seq = rng.permutation(np.repeat(np.arange(p["cols"]), p["rows"]))
    heights = [0] * p["cols"]
    bad, settle, tilts, ncon, ticks, never = [], [], [], 0, [], 0
    for i, col in enumerate(seq):
        dx, dy = rng.uniform(-offset, offset, 2)
        sim.place(i, int(col), drop_height(p), yaw=rng.uniform(0, 2 * np.pi), dx=dx, dy=dy)
        t = wait_settled(sim, i, ticks=ticks)
        ncon = max(ncon, sim.d.ncon)
        cell = sim.cell_of(i)
        expected = (int(col), heights[col])
        never += t is None
        if cell != expected:
            bad.append({"token": i, "expected": expected, "cell": cell, "t": t, "pos": sim.token_state(i)[0][:3].round(4).tolist()})
        heights[col] += 1
        settle.append(t if t is not None else np.nan)
        tilts.append(tilt_deg(sim, i))
    # Does the full stack hold still? Positions drift over 5 s of sim time.
    before = np.array([sim.token_state(i)[0][:3] for i in range(p["tokens"])])
    sim.step(int(5.0 / sim.m.opt.timestep))
    after = np.array([sim.token_state(i)[0][:3] for i in range(p["tokens"])])
    vmax = max(float(np.linalg.norm(sim.token_state(i)[1][:3])) for i in range(p["tokens"]))
    rows = [sim.token_state(i)[0][2] - tz for i in range(p["tokens"])]
    return {
        "bad": bad,
        "settle_p50": float(np.nanmedian(settle)),
        "settle_max": float(np.nanmax(settle)),
        "tilt_max": max(tilts),
        "drift_mm": float(np.abs(after - before).max() * 1e3),
        "vmax_mm_s": vmax * 1e3,
        "ticks": ticks,
        "never": never,
        "settle": settle,
        "ncon_max": ncon,
        "z_top_mm": float(max(rows) * 1e3),
    }


CASES = {
    "base (solref 4 ms)": {},
    "gap 8.5": {"gap": 0.0085},
    "gap 10.5": {"gap": 0.0105},
    "solref default 20 ms": {"solref": 0.02},
    "solref 10 ms": {"solref": 0.01},
    "margin 0.5 mm": {"margin": 0.0005},
    "friction 0.6": {"friction": 0.6},
    "mass 8 g": {"mass": 0.008},
    "condim 4": {"condim": 4},
    "timestep 1 ms": {"timestep": 0.001},
    "damping 0.05": {"damping": 0.05},
    "elliptic impratio 10": {"cone": "elliptic", "impratio": 10.0},
    "noslip 3": {"noslip": 3},
    "guide 30": {"guide_open": 0.03},
    "guide 40": {"guide_open": 0.04},
    "guide 30 friction 0.2": {"guide_open": 0.03, "friction": 0.2},
    "game options": {"solmix": 1000.0, "cone": "elliptic", "impratio": 100.0, "guide_open": 0.03, "park": "sleep", "sleep_tol": 1e-2, "damping": 0.005, "riser": 0.10},
    "game gap 8.5": {**{"solmix": 1000.0, "cone": "elliptic", "impratio": 100.0, "guide_open": 0.03, "park": "sleep", "sleep_tol": 1e-2, "damping": 0.005, "riser": 0.10}, "gap": 0.0085},
    "game gap 10.5": {**{"solmix": 1000.0, "cone": "elliptic", "impratio": 100.0, "guide_open": 0.03, "park": "sleep", "sleep_tol": 1e-2, "damping": 0.005, "riser": 0.10}, "gap": 0.0105},
}


def drop(args):
    results = {}
    names = args.only.split(",") if args.only else list(CASES)
    for name in names:
        over = CASES[name]
        games = [one_game(over, seed, offset=args.offset) for seed in range(args.games)]
        ticks = np.concatenate([g["ticks"] for g in games]) * 1e3
        settle = np.concatenate([g["settle"] for g in games])
        results[name] = {
            "tokens": 42 * len(games),
            "wrong": sum(len(g["bad"]) for g in games),
            "never_settled": sum(g["never"] for g in games),
            "settle_p50": round(float(np.nanmedian(settle)), 3),
            "settle_p95": round(float(np.nanpercentile(settle, 95)), 3),
            "settle_max": round(float(np.nanmax(settle)), 3),
            "tilt_max": round(max(g["tilt_max"] for g in games), 2),
            "drift_5s_mm": round(max(g["drift_mm"] for g in games), 3),
            "vmax_mm_s": round(max(g["vmax_mm_s"] for g in games), 3),
            "tick_ms_p50": round(float(np.median(ticks)), 2),
            "tick_ms_p99": round(float(np.percentile(ticks, 99)), 2),
            "tick_ms_max": round(float(ticks.max()), 2),
            "ncon_max": max(g["ncon_max"] for g in games),
            "first_bad": [b for g in games for b in g["bad"]][:2],
        }
        print(name, results[name], flush=True)
    return results


# ------------------------------------------------------------------------------------------------- (c)
ARM5 = ARM[:5]
GRIP_RANGE = (-0.17453297762778586, 1.7453291995659765)


def pct(p):
    return GRIP_RANGE[0] + p / 100.0 * (GRIP_RANGE[1] - GRIP_RANGE[0])


def to_pct(rad):
    return (rad - GRIP_RANGE[0]) / (GRIP_RANGE[1] - GRIP_RANGE[0]) * 100.0


class Rig(Sim):
    """Sim plus what the driver and the brain do around a held token."""

    def __init__(self, path, p, tz):
        super().__init__(path, p, tz)
        self.gripper = self.m.body("gripper").id
        self.site = self.m.site("gripperframe").id
        self.jaw_geoms = {g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] == self.m.body("moving_jaw_so101_v1").id and self.m.geom_contype[g]}
        self.fixed_geoms = {g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] == self.gripper and self.m.geom_contype[g]}
        self.floating = None
        self.setpoint = dict(zip(ARM5, [0.0] * 5, strict=True))

    def go_home(self, q5, g_pct):
        """Teleport the arm, no motion, as at startup."""
        for n, v in zip(ARM5, q5, strict=True):
            self.d.qpos[self.jq[n]] = v
            self.d.ctrl[self.act[n]] = v
            self.setpoint[n] = v
        self.d.qpos[self.jq["gripper"]] = pct(g_pct)
        self.d.ctrl[self.act["gripper"]] = pct(g_pct)
        self.d.qvel[:] = 0
        self.step(200)

    def tcp_frame(self):
        return self.d.site_xpos[self.site].copy(), self.d.xmat[self.gripper].reshape(3, 3).copy()

    def token_in_gripper(self, i):
        q = self.token_state(i)[0]
        R = self.d.xmat[self.gripper].reshape(3, 3)
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, q[3:7])
        R_t = mat.reshape(3, 3)
        return R.T @ (q[:3] - self.d.xpos[self.gripper]), R.T @ R_t

    def give(self, i, along=0.0, across=0.0, center=(0.0, 0.0, 0.0)):
        """give_token: axis on the jaw axis (gripper x), gravity compensated until the jaws pinch it."""
        pos, R = self.tcp_frame()
        R_token = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])  # token z = gripper x
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R_token.flatten())
        offset = np.asarray(center) + [across, 0, along]
        q, v = self.token_adr(i)
        self.d.qpos[q : q + 3] = pos + R @ offset
        self.d.qpos[q + 3 : q + 7] = quat
        self.d.qvel[v : v + 6] = 0
        self.d.qfrc_applied[v + 2] = self.p["mass"] * 9.81
        self.floating = i

    def touching(self, i):
        """Pinched: the token touches both jaws."""
        token = self.m.geom(f"token_{i}").id
        others = {c.geom2 if c.geom1 == token else c.geom1 for c in self.d.contact[: self.d.ncon] if token in (c.geom1, c.geom2)}
        return bool(others & self.jaw_geoms) and bool(others & self.fixed_geoms)

    def control_tick(self, goal, g_goal, v=1.0, gv=100.0):
        dt = 0.02
        for n in ARM5:
            e = goal[n] - self.setpoint[n]
            self.setpoint[n] += np.clip(e, -v * dt, v * dt)
            self.d.ctrl[self.act[n]] = self.setpoint[n]
        cur = self.d.ctrl[self.act["gripper"]]
        rate = gv / 100.0 * (GRIP_RANGE[1] - GRIP_RANGE[0])
        self.d.ctrl[self.act["gripper"]] = cur + np.clip(pct(g_goal) - cur, -rate * dt, rate * dt)
        self.tick()
        if self.floating is not None and self.touching(self.floating):
            _, vadr = self.token_adr(self.floating)
            self.d.qfrc_applied[vadr + 2] = 0.0
            self.floating = None

    def effort(self):
        a = self.act["gripper"]
        return float(-self.d.actuator_force[a] / self.m.actuator_forcerange[a][1])

    def grip_pct(self):
        return to_pct(self.d.qpos[self.jq["gripper"]])

    def goto(self, q5, g_goal, v=1.0, gv=100.0, extra=0.3, watch=None):
        goal = dict(zip(ARM5, q5, strict=True))
        T = max(abs(goal[n] - self.setpoint[n]) for n in ARM5) / v
        for _ in range(int((T + extra) / 0.02) + 1):
            self.control_tick(goal, g_goal, v, gv)
            if watch:
                watch()


HANDOVER = (0.9, -0.5, 0.5, 0.6)  # a candidate: pan 0.9 rad off the grid plane, jaws horizontal once rolled


def grip_trial(roll, over, along=0.0, across=0.0, target=0.0, gv=25.0, open_pct=40.0, seed_center=(0.0, 0.0, 0.0), speed=1.0, hold_s=3.0):
    path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
    rig = Rig(path, p, tz)
    q0 = (*HANDOVER, roll)
    rig.go_home(q0, open_pct)
    rig.give(0, along=along, across=across, center=seed_center)
    start = rig.token_state(0)[0][:3].copy()
    contacts0 = rig.d.ncon
    rig.step(5)
    before_close = np.linalg.norm(rig.token_state(0)[0][:3] - start)
    goal = dict(zip(ARM5, q0, strict=True))
    t_contact = None
    for k in range(int((open_pct - target) / gv / 0.02) + 25):
        rig.control_tick(goal, target, gv=gv)
        if rig.floating is None and t_contact is None:
            t_contact = (k + 1) * 0.02
    p0, R0 = rig.token_in_gripper(0)
    stop_pct, eff = rig.grip_pct(), rig.effort()
    hold_force = float(rig.d.actuator_force[rig.act["gripper"]])
    for _ in range(int(hold_s / 0.02)):  # the arm still, as during a human's turn
        rig.control_tick(goal, target, gv=gv)
    p1 = rig.token_in_gripper(0)[0]
    creep = float(np.linalg.norm(p1 - p0)) / hold_s
    p0, R0 = rig.token_in_gripper(0)
    # 1 rad/s out and back through the joints of a real move, then a stop.
    slips, spin = [], []
    def watch():
        pos, Rr = rig.token_in_gripper(0)
        slips.append(np.linalg.norm(pos - p0))
        spin.append(np.degrees(np.arccos(np.clip((np.trace(R0.T @ Rr) - 1) / 2, -1, 1))))
    for q in [(-0.2, -0.2, 0.2, 0.9, roll), (0.9, 0.2, -0.4, 1.2, roll), (-0.4, -0.6, 0.7, 0.4, roll), q0]:
        rig.goto(q, target, v=speed, watch=watch)
    z = rig.token_state(0)[0][2]
    pf, Rf = rig.token_in_gripper(0)
    held = np.linalg.norm(pf - p0) < 0.03
    return {
        "roll": round(roll, 2), "along_mm": along * 1e3, "across_mm": across * 1e3,
        "ncon_at_give": contacts0, "moved_before_close_mm": round(float(before_close) * 1e3, 3),
        "t_contact_s": t_contact, "stop_pct": round(float(stop_pct), 2), "effort": round(eff, 3),
        "gripper_force_Nm": round(hold_force, 3),
        "token_in_gripper_mm": (p0 * 1e3).round(2).tolist(),
        "creep_still_mm_s": round(creep * 1e3, 3),
        "slip_max_mm": round(float(max(slips)) * 1e3, 2), "spin_max_deg": round(float(max(spin)), 2),
        "held": bool(held), "z_end": round(float(z), 3),
    }


def grip(args):
    results = []
    pad = (PARAMS["t"] / 2 + 0.0005, 0.0, 0.0)
    configs = {
        "provided": {},
        "solmix": {"solmix": 1000.0},
        "game": GAME_OPTIONS,
        "game ir10": {**GAME_OPTIONS, "impratio": 10.0},
    }
    show = lambda res: print({k: v for k, v in res.items() if k not in ("along_mm", "across_mm", "ncon_at_give", "moved_before_close_mm")}, flush=True)
    for name, over in configs.items():
        for target in (0.0, 2.0, 3.0):
            res = {"cfg": name, "target": target, **grip_trial(np.pi / 2, over, seed_center=pad, target=target)}
            results.append(res)
            show(res)
    for roll in (np.pi / 2, -np.pi / 2):
        for along, across in ((0.003, 0.0), (-0.003, 0.0), (0.0, 0.005), (0.003, 0.005), (-0.003, 0.005)):
            res = {"cfg": "offsets", "target": 2.0, **grip_trial(roll, GAME_OPTIONS, along=along, across=across, seed_center=pad, target=2.0)}
            results.append(res)
            show(res)
    for speed in (0.5, 1.0, 1.5):
        res = {"cfg": f"speed {speed}", "target": 2.0, **grip_trial(np.pi / 2, GAME_OPTIONS, seed_center=pad, target=2.0, speed=speed)}
        results.append(res)
        show(res)
    for gv in (25.0, 100.0, 200.0):
        res = {"cfg": f"close {gv} %/s", "target": 2.0, **grip_trial(np.pi / 2, GAME_OPTIONS, seed_center=pad, target=2.0, gv=gv)}
        results.append(res)
        show(res)
    for fr in (0.3, 0.6):
        res = {"cfg": f"token friction {fr}", "target": 2.0, **grip_trial(np.pi / 2, {**GAME_OPTIONS, "friction": fr}, seed_center=pad, target=2.0)}
        results.append(res)
        show(res)
    for label, across in (("on the TCP", 0.0), ("on the TCP, -5 mm depth (#80 as written)", -0.005)):
        res = {"cfg": label, "target": 2.0, **grip_trial(np.pi / 2, GAME_OPTIONS, target=2.0, across=across)}
        results.append(res)
        show(res)
    return results


# ------------------------------------------------------------------------------------------------ (c+)
# Full move of the robot: handover, pinch, move, descent, release, the token ends in its cell.
GAME_OPTIONS = {"solmix": 1000.0, "cone": "elliptic", "impratio": 100.0, "guide_open": 0.03, "park": "sleep", "sleep_tol": 1e-2, "damping": 0.005, "riser": 0.10}
PAD = (PARAMS["t"] / 2 + 0.0005, 0.0, 0.0)  # token center from the TCP, gripper frame: on the fixed jaw's face


def ik(rig, target, roll, q_seed, p_g, tilt=0.0):
    """Token center p_g (gripper frame) at target, roll locked, pan+lift+elbow+flex; DLS on a data copy.
    4th row: the approach axis `tilt` rad from vertical inside the arm plane, weak: as vertical as reachable."""
    m = rig.m
    d = mujoco.MjData(m)
    d.qpos[:] = rig.d.qpos
    dofs = [m.joint(n).dofadr[0] for n in ARM5[:4]]
    q = np.array(q_seed, dtype=float)
    low = m.jnt_range[[m.joint(n).id for n in ARM5[:4]], 0]
    high = m.jnt_range[[m.joint(n).id for n in ARM5[:4]], 1]
    for it in range(400):
        for n, v in zip(ARM5[:4], q, strict=True):
            d.qpos[rig.jq[n]] = v
        d.qpos[rig.jq["wrist_roll"]] = roll
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        R = d.xmat[rig.gripper].reshape(3, 3)
        p = d.xpos[rig.gripper] + R @ np.asarray(p_g)
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jacp, jacr, p, rig.gripper)
        e = target - p
        n_vec = R[:, 0]
        # gripper z towards world z, about the jaw axis
        ang = np.arctan2(np.cross(R[:, 2], [0, 0, 1]) @ n_vec, R[:, 2] @ [0, 0, 1]) - tilt
        J = jacp[:, dofs]
        Jinv = J.T @ np.linalg.inv(J @ J.T + 1e-5 * np.eye(3))
        null = 0.0 if it > 300 or np.linalg.norm(e) > 1e-3 else 0.2  # the tilt only once on target, last 100: position
        dq = Jinv @ e + (np.eye(4) - Jinv @ J) @ (null * (n_vec @ jacr[:, dofs]) * ang)
        q = np.clip(q + 0.5 * dq, low, high)
    return q, float(np.linalg.norm(e)), float(np.degrees(ang)), float(np.degrees(np.arccos(np.clip(abs(R[:, 0] @ [-np.sin(rig.p["az"]), np.cos(rig.p["az"]), 0]), -1, 1))))


NOMINAL_GRIP = np.array([-0.00516, -0.00012, -0.09785])  # token center in the gripper frame, measured once: all the brain knows
REACH = np.load(Path(__file__).parent / "reach.npy") if (Path(__file__).parent / "reach.npy").exists() else None  # reach.py


def full_move(rig, col, row, roll, target=2.0, gv=25.0, open_pct=40.0, tcp_err=(0.0, 0.0), hand=(0.0, 0.0), release_gv=100.0, token=0, grip_offset=None, height=0.005):
    """One robot move; returns (cell or None, facts)."""
    p = rig.p
    q_hand = (*HANDOVER, roll)
    rig.goto(q_hand, open_pct, v=1.0, gv=200.0)
    rig.give(token, along=hand[0], across=hand[1], center=PAD)
    goal = dict(zip(ARM5, q_hand, strict=True))
    for _ in range(int((open_pct - target) / gv / 0.02) + 25):
        rig.control_tick(goal, target, gv=gv)
    p_g, _ = rig.token_in_gripper(token)
    effort = rig.effort()
    # where the token really is in the gripper is what the brain does not know: it plans with the nominal offset.
    p_plan = p_g if grip_offset is None else grip_offset
    a = p["az"]
    along_v, across_v = np.array([np.cos(a), np.sin(a), 0.0]), np.array([-np.sin(a), np.cos(a), 0.0])
    xy = rig.column_xy(col)
    base = np.array([xy[0], xy[1], rig.table_z + p["entry"] + p["guide_h"] + p["d"] / 2])
    base = base + tcp_err[0] * along_v + tcp_err[1] * across_v
    pre = base + [0, 0, 0.03]
    best = None
    near = REACH[np.hypot(REACH[:, 0] - pre[0], REACH[:, 2] - pre[2]) < 0.01]
    for seed in near[np.argsort(near[:, 3])[:12]]:  # the most vertical samples around the target
        sol = ik(rig, pre, roll, (0.0, *seed[4:7]), p_plan)
        if best is None or (sol[1] > 5e-4, abs(sol[2])) < (best[1] > 5e-4, abs(best[2])):
            best = sol
    q_pre, e1, a1, j1 = best
    q_drop, e2, a2, j2 = ik(rig, base + [0, 0, height], roll, q_pre, p_plan)
    facts = {"ik_err_mm": round(max(e1, e2) * 1e3, 2), "approach_deg": round(a2, 1), "jaw_err_deg": round(j2, 2), "effort": round(effort, 3)}
    if max(e1, e2) > 0.002:
        return None, {**facts, "fail": "unreachable"}
    rig.goto((HANDOVER[0], *q_pre[1:], roll), target, v=1.0, gv=gv)  # up first: HANDOVER is below the grid top
    rig.goto((*q_pre, roll), target, v=1.0, gv=gv)
    rig.goto((*q_drop, roll), target, v=0.3, gv=gv)
    actual = rig.token_state(token)[0][:3]
    facts["pos_err_mm"] = (np.array(actual - base - [0, 0, height]) * 1e3).round(1).tolist()
    goal = dict(zip(ARM5, (*q_drop, roll), strict=True))
    for _ in range(50):
        rig.control_tick(goal, open_pct, gv=release_gv)
    t = wait_settled(rig, token, timeout=2.0)
    rig.goto((*q_pre, roll), open_pct, v=1.0, gv=200.0)
    rig.goto((HANDOVER[0], *q_pre[1:], roll), open_pct, v=1.0, gv=200.0)
    rig.goto(q_hand, open_pct, v=1.0, gv=200.0)
    return rig.cell_of(token), facts


def move(args):
    """All columns, both rolls, nominal; then --n moves with the errors of #76: hand +-3 mm along, 0 to 5 mm in depth,
    TCP +-args.offset both ways. --game: a whole game, robot + simulated human, on one MjData."""
    over = dict(GAME_OPTIONS) | {k: float(v) for k, v in (kv.split("=") for kv in args.set.split(",") if kv)}
    results = []
    rng = np.random.default_rng(0)
    roll = np.pi / 2
    if args.n == 0:
        for roll in (np.pi / 2, -np.pi / 2):
            path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
            rig = Rig(path, p, tz)
            rig.go_home((*HANDOVER, roll), 40.0)
            for col in range(p["cols"]):
                cell, facts = full_move(rig, col, 0, roll, token=col)
                res = {"roll": round(roll, 2), "col": col, "cell": cell, "ok": cell == (col, 0), **facts}
                results.append(res)
                print(res, flush=True)
    else:
        path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
        rig = Rig(path, p, tz)
        rig.go_home((*HANDOVER, roll), 40.0)
        heights = [0] * 7
        for k in range(args.n):
            col = int(rng.integers(7))
            if heights[col] >= 6:
                continue
            hand = (rng.uniform(-0.003, 0.003), rng.uniform(0.0, 0.005))
            err = tuple(rng.uniform(-args.offset, args.offset, 2))
            t0 = time.perf_counter()
            cell, facts = full_move(rig, col, heights[col], roll, token=sum(heights), hand=hand, tcp_err=err, grip_offset=NOMINAL_GRIP if args.blind else None)
            ok = cell == (col, heights[col])
            res = {"k": k, "col": col, "row": heights[col], "cell": cell, "ok": ok, "hand_mm": np.round(np.array(hand) * 1e3, 1).tolist(), "tcp_err_mm": np.round(np.array(err) * 1e3, 1).tolist(), "wall_s": round(time.perf_counter() - t0, 1), **facts}
            results.append(res)
            print(res, flush=True)
            if ok:
                heights[col] += 1
            else:  # a lost token is out of the count: bring it back to its parking place
                pass
    ok = sum(r["ok"] for r in results)
    print("moves ok", ok, "/", len(results), flush=True)
    return results


# ------------------------------------------------------------------------------------------------- (d)
def repark(args):
    """Every token back to its parking place, between two ticks. Timed as the driver's service would run."""
    results = {}
    for name, over in (("game options", GAME_OPTIONS), ("sleep", {**GAME_OPTIONS, "park": "sleep"})):
        path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
        sim = Sim(path, p, tz)
        sim.home((-1.5, -1.2, 1.2, 0.3, 0.0, 1.0))
        sim.step(300)
        sim.fill([6] * 7)
        adr = np.array([sim.token_adr(i) for i in range(p["tokens"])])
        parked = np.array([[*park_xy(i), tz + p["t"] / 2, 1, 0, 0, 0] for i in range(p["tokens"])])
        qidx = (adr[:, 0, None] + np.arange(7)).ravel()
        vidx = (adr[:, 1, None] + np.arange(6)).ravel()
        qarm = sim.d.qpos[[sim.jq[n] for n in ARM]].copy()
        walls, forwards = [], []
        for _ in range(20):
            sim.fill([6] * 7, settle=0.2)
            t0 = time.perf_counter()
            sim.d.qpos[qidx] = parked.ravel()
            sim.d.qvel[vidx] = 0
            sim.d.qfrc_applied[vidx] = 0
            t1 = time.perf_counter()
            mujoco.mj_forward(sim.m, sim.d)
            t2 = time.perf_counter()
            walls.append(t1 - t0)
            forwards.append(t2 - t1)
        # the first ticks after: do the parked tokens hold still, does the arm feel anything
        ticks = []
        for _ in range(50):
            t0 = time.perf_counter()
            sim.tick()
            ticks.append(time.perf_counter() - t0)
        speed = max(float(np.linalg.norm(sim.token_state(i)[1][:3])) for i in range(p["tokens"]))
        moved = max(float(np.linalg.norm(sim.token_state(i)[0][:2] - park_xy(i))) for i in range(p["tokens"]))
        arm = float(np.abs(sim.d.qpos[[sim.jq[n] for n in ARM]] - qarm).max())
        r = {
            "write_us_p50": round(float(np.median(walls)) * 1e6, 1),
            "write_us_max": round(float(max(walls)) * 1e6, 1),
            "mj_forward_ms_p50": round(float(np.median(forwards)) * 1e3, 3),
            "first_50_ticks_ms_max": round(float(max(ticks)) * 1e3, 2),
            "parked_speed_mm_s": round(speed * 1e3, 3),
            "parked_moved_mm": round(moved * 1e3, 3),
            "arm_moved_rad": round(arm, 5),
        }
        # a new game right after: one token into column 3, from the parking place
        sim.place(0, 3, drop_height(p))
        t = wait_settled(sim, 0)
        r["drop_after_repark"] = (sim.cell_of(0), t)
        results[name] = r
        print(name, r, flush=True)
    return results


# ------------------------------------------------------------------------------------------------- riser
def riser(args):
    """What each way of modelling the riser does to the assumptions of the driver and the tests."""
    import sys as _s

    from so101_sim import SO101Sim

    out = {}
    for mode in ("raise", "lower"):
        path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), riser_mode=mode, **GAME_OPTIONS)
        sim = SO101Sim(scene_path=path, camera_width=320, camera_height=240, seed=1)
        sim.connect()
        m, d = sim.model, sim.data
        base = d.body("base").xpos.copy()
        tcp = d.site("gripperframe").xpos.copy()
        cam = sim.get_camera_extrinsics()
        cube0 = d.body("pickup_object").xpos.copy()
        cube_ncon = sum(1 for c in d.contact[: d.ncon] if m.geom_bodyid[c.geom1] == m.body("pickup_object").id or m.geom_bodyid[c.geom2] == m.body("pickup_object").id)
        for _ in range(200):
            sim.get_observation()
        cube1 = d.body("pickup_object").xpos.copy()
        out[mode] = {
            "base_xpos": base.round(4).tolist(),
            "table_z_world": tz,
            "tcp_minus_base": (tcp - base).round(4).tolist(),
            "camera_z_world": round(float(cam[2, 3]), 4),
            "cube_after_connect": cube0.round(3).tolist(),
            "cube_after_4s": cube1.round(3).tolist(),
            "cube_contacts": cube_ncon,
            "drop_box": d.body("drop_box").xpos.round(3).tolist(),
        }
        print(mode, out[mode], flush=True)
        sim.disconnect()
    return out


# ------------------------------------------------------------------------------------------------- (a2)
def tick_cost(over, label, ticks=300):
    """CPU time of one tick (10 mj_step, this thread), four states of a game. thread_time: other loads do not count."""
    res = {}
    for state in ("parked", "21 settled", "42 settled", "drops into 20"):
        path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
        sim = Sim(path, p, tz)
        sim.home((-1.5, -1.2, 1.2, 0.3, 0.0, 1.0))
        sim.step(100)
        cols = {"parked": [0] * 7, "21 settled": [3] * 7, "42 settled": [6] * 7, "drops into 20": [3, 3, 3, 3, 3, 3, 2]}[state]
        n = sim.fill(cols)
        times, ncon, nefc, solver = [], [], [], []
        for k in range(ticks):
            if state == "drops into 20" and k % 40 == 0:  # a token falls every 0.8 s, on a board that fills
                col = k // 40 % 7
                sim.place(n + k // 40 if n + k // 40 < 42 else 41, col, drop_height(p), yaw=0.3 * k)
            arm_motion(sim, k * 0.02)
            t0 = time.thread_time()
            sim.tick()
            times.append(time.thread_time() - t0)
            ncon.append(sim.d.ncon)
            nefc.append(sim.d.nefc)
            solver.append(int(sim.d.solver_niter[0]))
        t = np.array(times) * 1e3
        res[state] = {
            "p50": round(float(np.median(t)), 2), "p99": round(float(np.percentile(t, 99)), 2), "max": round(float(t.max()), 2),
            "ncon": int(np.median(ncon)), "ncon_max": int(max(ncon)), "nefc_max": int(max(nefc)), "iter_max": int(max(solver)),
        }
    print(label, res, flush=True)
    return res


def cost2(args):
    grip = {"solmix": 1000.0, "guide_open": 0.03}
    configs = {
        "final": dict(GAME_OPTIONS),
        "final, no sleep": {**GAME_OPTIONS, "park": "floor"},
        "pyramidal": {"guide_open": 0.03},
        "elliptic ir10": {**grip, "cone": "elliptic", "impratio": 10.0},
        "elliptic ir100": {**grip, "cone": "elliptic", "impratio": 100.0},
        "elliptic ir10 noslip5": {**grip, "cone": "elliptic", "impratio": 10.0, "noslip": 5},
        "elliptic ir100 tol1e-6 it30": {**grip, "cone": "elliptic", "impratio": 100.0, "tolerance": 1e-6, "iterations": 30},
        "elliptic ir100 sleep": {**grip, "cone": "elliptic", "impratio": 100.0, "park": "sleep"},
    }
    return {name: tick_cost(over, name) for name, over in configs.items()}


# ------------------------------------------------------------------------------------------------- (a3)
def settle(args):
    """A game of drops, one token each 0.6 s: do the tokens go to sleep, what does a tick cost then?"""
    base = dict(GAME_OPTIONS)
    configs = {
        "d0 tol 1e-1": {"damping": 0.0, "sleep_tol": 1e-1},
        "d0 tol 3e-1": {"damping": 0.0, "sleep_tol": 3e-1},
        "d.002 tol 1e-2": {"damping": 0.002, "sleep_tol": 1e-2},
        "d.002 tol 3e-2": {"damping": 0.002, "sleep_tol": 3e-2},
        "d.005 tol 1e-2": {"damping": 0.005, "sleep_tol": 1e-2},
        "d.01 tol 1e-2": {"damping": 0.01, "sleep_tol": 1e-2},
        "tol 1e-3": {},
        "tol 1e-2": {"sleep_tol": 1e-2},
        "tol 3e-2": {"sleep_tol": 3e-2},
        "tol 1e-1": {"sleep_tol": 1e-1},
        "no multiccd": {"multiccd": False},
        "no multiccd tol 1e-2": {"multiccd": False, "sleep_tol": 1e-2},
        "inertia x10": {"inertia_scale": 10.0},
        "inertia x100": {"inertia_scale": 100.0},
        "damping 0.02": {"damping": 0.02},
        "damping 0.02 tol 1e-2": {"damping": 0.02, "sleep_tol": 1e-2},
        "friction 0.6 tol 1e-2": {"friction": 0.6, "sleep_tol": 1e-2},
        "ir10 tol 1e-2": {"impratio": 10.0, "sleep_tol": 1e-2},
        "ir10 inertia x10 tol 1e-2": {"impratio": 10.0, "sleep_tol": 1e-2, "inertia_scale": 10.0},
    }
    if args.only:
        configs = {k: v for k, v in configs.items() if k in args.only.split(",")}
    out = {}
    for name, over in configs.items():
        path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **(base | over))
        sim = Sim(path, p, tz)
        sim.home((-1.5, -1.2, 1.2, 0.3, 0.0, 1.0))
        sim.step(100)
        rng = np.random.default_rng(args.seed)
        seq = rng.permutation(np.repeat(np.arange(7), 6))
        heights = [0] * 7
        wrong, cpu, awake_trace = 0, [], []
        for i, col in enumerate(seq):
            sim.place(i, int(col), drop_height(p), yaw=rng.uniform(0, 6.28), dx=rng.uniform(-0.002, 0.002), dy=rng.uniform(-0.002, 0.002))
            for _ in range(30):
                t0 = time.thread_time()
                sim.tick()
                cpu.append(time.thread_time() - t0)
            sim.step(100)
            wrong += sim.cell_of(i) != (int(col), heights[col])
            heights[col] += 1
            if i % 7 == 6:
                awake_trace.append(int(sum(sim.d.body_awake[sim.m.body(f"token_{k}").id] != 0 for k in range(42))))
        tail = []
        sim.step(1000)
        for _ in range(100):
            t0 = time.thread_time()
            sim.tick()
            tail.append(time.thread_time() - t0)
        c = np.array(cpu) * 1e3
        v = max(float(np.linalg.norm(sim.token_state(k)[1][:3])) for k in range(42)) * 1e3
        out[name] = {"wrong": int(wrong), "awake_every_7": awake_trace, "game_tick_p50": round(float(np.median(c)), 2), "p99": round(float(np.percentile(c, 99)), 2), "max": round(float(c.max()), 2),
                     "full_board_tick": round(float(np.median(tail)) * 1e3, 2), "v_mm_s": round(v, 2)}
        print(name, out[name], flush=True)
    return out


def capture(args):
    """TCP error sweep on column 3, one axis at a time, roll +90 deg: the capture of the drop (#76)."""
    over = dict(GAME_OPTIONS) | {k: float(v) for k, v in (kv.split("=") for kv in args.set.split(",") if kv)}
    out = []
    for axis, values in (("along", (-20, -16, -12, -8, -4, 4, 8, 12, 16, 20)), ("across", (-14, -12, -10, -8, -6, 6, 8, 10, 12, 14))):
        for mm in values:
            path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **over)
            rig = Rig(path, p, tz)
            rig.go_home((*HANDOVER, np.pi / 2), 40.0)
            err = (mm / 1e3, 0.0) if axis == "along" else (0.0, mm / 1e3)
            cell, facts = full_move(rig, 3, 0, np.pi / 2, token=0, tcp_err=err)
            out.append({"axis": axis, "mm": mm, "cell": cell, "ok": cell == (3, 0)})
            print(out[-1], flush=True)
    return out


# ------------------------------------------------------------------------------------------------- #76
# One layout for all the numbers (#76): P2-02 §7 (connect4_grid.yaml), with the drop pose of drop/e7_cycle.py
# through the brain's Kinematics. Earlier commands keep their own layouts, as they ran for MES-63.
LAYOUT = GAME_OPTIONS | {
    "az": np.radians(-40.0),  # [MES-65]
    "plane": -0.00385,  # the slot plane 3.85 mm off the pan axis, on the side of the TCP line [MES-64]
    "guide_h": 0.030,  # [MES-63]
    "camera": None,  # the phase-1 camera, unmoved [MES-65]
}
HANDOVER_Q = (-1.0, -0.6, 0.6, 1.1, 1.57)  # P2-02 §7's candidate: pan <= -0.7 hides nothing from the camera
ROLL = 1.6195  # rad, the jaws normal to the grid [MES-64]; so101_brain/drop.py (#83)
TILT = 70.0  # deg, the approach in the arm plane [MES-64]
TOKEN_IN_TCP = np.array([-0.00315, 0.0, -0.001])  # m, the held token's centre in gripper_frame_link [MES-64]
# m: the token centre aimed off the slot plane towards the fixed jaw (P2-02 §2.2). The token rests on it, at -x of
# the TCP; at azimuth a the jaws' x is (sin a, -cos a, 0) = -normal, so the fixed jaw is on the +normal side.
AIM = 0.001
RELEASE_ABOVE = 0.008  # m, the token's bottom above the guide at the release (P2-02 §2.3)
PRE_DROP = 0.025  # m above the release: height first, then a joint descent (at 30 mm, column 6 is near its reach)
OPEN, SQUEEZE = 40.0, 0.0  # gripper % at HANDOVER and closed on the token [MES-64]
DESCENT_V = 0.3  # rad/s
# m above the release: the lift goes straight up through IK waypoints. One joint move from the release to the
# pre-drop swept the fixed jaw into the guide: 0.0 mm at column 6, 15.3 mm like this (#76)
LIFT_STEPS = (0.005, 0.01, 0.015, 0.02, PRE_DROP)
_KIN = []
_AIM = []  # --aim-mm


def aim():
    return _AIM[0] if _AIM else AIM


def brain_kinematics():
    """so101_brain's Kinematics on the URDF the workspace installs (tools/so101 run builds it)."""
    if not _KIN:
        from ament_index_python.packages import get_package_share_directory
        from so101_brain.kinematics import Kinematics

        _KIN.append(Kinematics(str(Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf")))
    return _KIN[0]


def release_pose(target, q_seed):
    """drop/e7_cycle.py's rel_pose, as so101_brain/drop.py (#83): the joints that put the token's centre on target."""
    from so101_brain.grasp_planner import tilted_down

    kin = brain_kinematics()
    azimuth, seed = kin.arm_azimuth(target), np.asarray(q_seed, dtype=float)
    for _ in range(2):
        z = tilted_down(TILT, azimuth)
        x = np.array([np.sin(azimuth), -np.cos(azimuth), 0.0])
        p = target - np.column_stack([x, np.cross(z, x), z]) @ TOKEN_IN_TCP
        q = kin.ik_pos(p, q_seed=seed, roll=ROLL)
        if q is None:
            return None
        azimuth, seed = -q[0], q
    return kin.ik_point(p, tilted_down(TILT, azimuth), q_seed=seed, roll=ROLL)


class Clearance:
    """The smallest distance between the arm's geoms and the depth guide's, over the ticks it watches."""

    def __init__(self, rig):
        m = rig.m
        base = m.body("base").id

        def on_arm(b):
            while b:
                if b == base:
                    return True
                b = m.body_parentid[b]
            return False

        self.rig = rig
        self.arm = [g for g in range(m.ngeom) if on_arm(m.geom_bodyid[g]) and m.geom_contype[g]]
        self.guide = [m.geom(f"guide_{s}").id for s in ("front", "back")]
        self.min, self.where, self.phase = np.inf, "", ""

    def __call__(self):
        m, d = self.rig.m, self.rig.d
        for a in self.arm:
            for g in self.guide:
                dist = mujoco.mj_geomDistance(m, d, a, g, 0.05, None)
                if dist < self.min:
                    body = m.body(m.geom_bodyid[a]).name
                    self.min, self.where = dist, f"{body}/{m.geom(a).name or a} in {self.phase}"


def robot_move(rig, col, token, hand=(0.0, 0.0), tcp_err=(0.0, 0.0)):
    """One move of /brain/drop on the layout: HANDOVER, give_token with `hand` (along the grid, in depth, m), close,
    pre-drop height first, a joint descent to the release, open, back. `tcp_err` (along, across, m) moves the
    target, as a perception error would. Returns (cell or None, facts)."""
    p, a = rig.p, rig.p["az"]
    along_v, normal = np.array([np.cos(a), np.sin(a), 0.0]), np.array([-np.sin(a), np.cos(a), 0.0])
    xy = rig.column_xy(col)
    target = np.array([*xy, rig.table_z + p["entry"] + p["guide_h"] + RELEASE_ABOVE + p["d"] / 2])
    target = target + aim() * normal + tcp_err[0] * along_v + tcp_err[1] * normal
    rig.goto(HANDOVER_Q, OPEN, v=1.0, gv=200.0)
    pos, R = rig.tcp_frame()
    rig.give(token, center=(PAD[0], hand[0], hand[1]))  # gripper body frame: x the jaws, y along the grid, z depth
    goal = dict(zip(ARM5, HANDOVER_Q, strict=True))
    for _ in range(int(OPEN / 25.0 / 0.02) + 25):
        rig.control_tick(goal, SQUEEZE, gv=25.0)
    held = bool(rig.effort() > 0.06)  # the brain's grasp check in sim [MES-48]
    q_pre = release_pose(target + [0, 0, PRE_DROP], HANDOVER_Q)
    q_rel = release_pose(target, q_pre) if q_pre is not None else None
    facts = {"held": held}
    if q_rel is None:
        return None, {**facts, "fail": "unreachable"}
    clearance = Clearance(rig)
    rig.goto((HANDOVER_Q[0], *q_pre[1:]), SQUEEZE, v=1.0, gv=25.0)  # up first: HANDOVER is below the grid top
    clearance.phase = "to pre-drop"
    rig.goto(q_pre, SQUEEZE, v=1.0, gv=25.0, watch=clearance)
    clearance.phase = "descent"
    rig.goto(q_rel, SQUEEZE, v=DESCENT_V, gv=25.0, watch=clearance)
    facts["token_err_mm"] = round(float(np.linalg.norm(rig.token_state(token)[0][:3] - target)) * 1e3, 1)
    goal = dict(zip(ARM5, q_rel, strict=True))
    clearance.phase = "release"
    for _ in range(25):
        rig.control_tick(goal, OPEN, gv=200.0)
        clearance()
    settled = wait_settled(rig, token, timeout=2.0)
    clearance.phase = "lift"
    q = q_rel
    for h in LIFT_STEPS:  # straight up through IK waypoints: one joint move from the release swept the guide
        q = release_pose(target + [0, 0, h], q) if h < PRE_DROP else q_pre
        rig.goto(q if q is not None else q_pre, OPEN, v=1.0, gv=200.0, watch=clearance)
    rig.goto((HANDOVER_Q[0], *q_pre[1:]), OPEN, v=1.0, gv=200.0)
    rig.goto(HANDOVER_Q, OPEN, v=1.0, gv=200.0)
    z = rig.token_state(token)[0][2] - rig.table_z
    cell = rig.cell_of(token)
    if cell is not None and cell[1] >= p["rows"]:  # resting on the slot entry, above the grid: jammed
        cell = None
    facts |= {"clearance_mm": round(float(clearance.min) * 1e3, 1), "closest": clearance.where}
    facts |= {"settled": settled is not None, "jammed": bool(cell is None and z > bottom_z(p))}  # above the stack
    return cell, facts


def drops(n, tcp_mm, seed):
    """n moves into random columns, give_token's offsets (+-3 mm along, +-5 mm in depth) and +-tcp_mm on the
    target; a new scene when the grid is half full. Counts by outcome."""
    rng = np.random.default_rng(seed)
    out, rig, heights, k = [], None, None, 0
    while k < n:
        if rig is None or sum(heights) >= 21:
            path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **LAYOUT)
            rig, heights = Rig(path, p, tz), [0] * p["cols"]
            rig.go_home(HANDOVER_Q, OPEN)
        col = int(rng.choice([c for c in range(7) if heights[c] < 6]))
        hand = (rng.uniform(-0.003, 0.003), rng.uniform(-0.005, 0.005))
        err = tuple(rng.uniform(-tcp_mm, tcp_mm, 2) / 1e3)
        cell, facts = robot_move(rig, col, sum(heights), hand=hand, tcp_err=err)
        if facts.get("fail") == "unreachable":
            facts["target_r_mm"] = round((p["r0"] + p["pitch"] * col + err[0]) * 1e3, 1)
        ok = cell == (col, heights[col])
        out.append({"col": col, "ok": ok, "cell": cell, "hand_mm": (np.array(hand) * 1e3).round(1).tolist(),
                    "tcp_mm": (np.array(err) * 1e3).round(1).tolist(), **facts})
        if cell is not None:
            heights[cell[0]] += 1  # the camera wins: where it landed
        else:  # lost or jammed: out of the count, a new scene
            rig = None
        k += 1
    return out


def summary(name, moves):
    ok = sum(m["ok"] for m in moves)
    wrong = sum(m["cell"] is not None and not m["ok"] for m in moves)
    jammed = sum(m.get("jammed", False) for m in moves)
    unreachable = sum(m.get("fail") == "unreachable" for m in moves)
    lost = len(moves) - ok - wrong - jammed - unreachable
    print(f"{name}: {ok}/{len(moves)} in the right cell ({100 * ok / len(moves):.0f} %); wrong cell {wrong}, "
          f"jammed {jammed}, lost {lost}, unreachable {unreachable}; not held {sum(not m['held'] for m in moves)}",
          flush=True)
    return {"name": name, "ok": ok, "n": len(moves), "wrong": wrong, "jammed": jammed, "lost": lost,
            "unreachable": unreachable}


def spike(args):
    """#76 on the layout of P2-02 §7: the arm-to-guide clearance on the 7 columns, 100 drops with give_token's
    offsets, 100 with +-5 mm on the TCP too, the capture along and across. Go: >= 95 % and >= 80 %."""
    results = {}
    path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **LAYOUT)
    rig = Rig(path, p, tz)
    rig.go_home(HANDOVER_Q, OPEN)
    columns = []
    for col in range(p["cols"]):
        cell, facts = robot_move(rig, col, col)
        columns.append({"col": col, "cell": cell, **facts})
        print(columns[-1], flush=True)
    results["columns"] = columns
    print(f"clearance arm-guide, 7 columns: min {min(c['clearance_mm'] for c in columns)} mm", flush=True)
    for name, tcp in (("give_token offsets", 0.0), ("+-5 mm TCP on top", 5.0)):
        moves = drops(args.n or 100, tcp, args.seed)
        results[name] = {"summary": summary(name, moves), "moves": moves}
    sweep = []
    for axis, values in (("along", (-20, -16, -12, -8, -4, 0, 4, 8, 12, 16, 20)),
                         ("across", (-12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12))):
        for mm in values:
            path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **LAYOUT)
            rig = Rig(path, p, tz)
            rig.go_home(HANDOVER_Q, OPEN)
            err = (mm / 1e3, 0.0) if axis == "along" else (0.0, mm / 1e3)
            cell, facts = robot_move(rig, 3, 0, tcp_err=err)
            sweep.append({"axis": axis, "mm": mm, "ok": cell == (3, 0), "cell": cell, **facts})
    for axis in ("along", "across"):
        row = [s for s in sweep if s["axis"] == axis]
        zero = next(i for i, s in enumerate(row) if s["mm"] == 0)
        lo = hi = zero
        while lo > 0 and row[lo - 1]["ok"]:
            lo -= 1
        while hi < len(row) - 1 and row[hi + 1]["ok"]:
            hi += 1
        window = f"{row[lo]['mm']} to {row[hi]['mm']} mm" if row[zero]["ok"] else "none"
        print(f"capture {axis} (around the aimed point, unbroken): {window} "
              f"({''.join('o' if s['ok'] else '.' for s in row)}, {row[0]['mm']} to {row[-1]['mm']} mm)", flush=True)
    results["capture"] = sweep
    a, b = (results[k]["summary"] for k in ("give_token offsets", "+-5 mm TCP on top"))
    go = a["ok"] >= 0.95 * a["n"] and b["ok"] >= 0.80 * b["n"]
    print(f"{'GO' if go else 'NO-GO'}: {a['ok']}/{a['n']} (>= 95 %), {b['ok']}/{b['n']} with +-5 mm (>= 80 %); "
          f"aim {aim() * 1e3:+.1f} mm towards the fixed jaw", flush=True)
    return results


def read_stacks(rig):
    """The tokens where the moves left them, copied into reading/'s scene on the same layout (the phase-1 camera,
    640x480, the 30 mm markers, shadows 1024, no floor reflection), then read as reading/final.py does: the grid
    pose from the markers (PnP), a disc per cell, the HSV fractions. Returns (cells right, cells, markers)."""
    sys.path.insert(0, str(Path(__file__).parent.parent / "reading"))
    import common
    from classify import read

    common.TABLE_Z = rig.table_z  # reading/ lowered its floor by 8 cm, the layout by 10
    g = common.Grid(az=np.degrees(LAYOUT["az"]), marker=0.03, table=rig.table_z)
    g.t = g.t + LAYOUT["plane"] * g.y  # the slot plane off the pan axis, as the physics scene
    g.T[:3, 3] = g.t
    w = common.World(g, 640, 480, cam=common.P1_CAM, shadow=1024, reflect=0.0)
    w.arm(HANDOVER_Q)
    truth = np.zeros((common.ROWS, common.COLS), int)
    used = {"r": 0, "y": 0}
    for i in range(rig.p["tokens"]):
        cell = rig.cell_of(i)
        if cell is None:
            continue
        color = "r" if i % 2 == 0 else "y"  # the physics scene's colors: red even, yellow odd
        truth[cell[1], cell[0]] = 1 if color == "r" else 2
        a = w.tok_adr[f"{color}{used[color]}"]
        used[color] += 1
        w.d.qpos[a : a + 7] = rig.token_state(i)[0]
    rgb = w.render()
    found = common.pnp(common.detect(rgb), g, w.K)
    if found is None:
        w.close()
        return 0, truth.size, 0
    out = read(rgb, g, w.K, found[0], thr=0.35)[0]
    w.close()
    return int((out == truth).sum()), truth.size, len(found[2])


def stacks(args):
    """Full stacks (#76): the robot fills the grid through the guide, row by row (42 moves), then reading/'s method
    reads the 42 cells. --n: grids filled (default 2)."""
    results = []
    for k in range(args.n or 2):
        rng = np.random.default_rng(args.seed + k)
        path, p, tz = build(tempfile.mkdtemp(prefix="so101_"), **LAYOUT)
        rig = Rig(path, p, tz)
        rig.go_home(HANDOVER_Q, OPEN)
        moves = []
        for token in range(p["tokens"]):
            col = token % p["cols"]
            hand = (rng.uniform(-0.003, 0.003), rng.uniform(-0.005, 0.005))
            cell, facts = robot_move(rig, col, token, hand=hand)
            moves.append({"col": col, "ok": cell == (col, token // p["cols"]), "cell": cell, **facts})
        right, cells, markers = read_stacks(rig)
        full = sum(m["ok"] for m in moves)
        print(f"grid {k}: {full}/{len(moves)} tokens in their cell, jammed {sum(m['jammed'] for m in moves)}, "
              f"clearance min {min(m['clearance_mm'] for m in moves)} mm; read {right}/{cells} cells "
              f"({markers}/4 markers)", flush=True)
        results.append({"moves": moves, "read_right": right, "cells": cells, "markers": markers})
    return results


COMMANDS = {"spike": spike, "stacks": stacks, "capture": capture, "settle": settle, "cost": cost, "cost2": cost2, "scene": scene, "drop": drop, "grip": grip, "move": move, "repark": repark, "riser": riser}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=sorted(COMMANDS))
    parser.add_argument("out", nargs="?", default="/tmp/c4")
    parser.add_argument("--json")
    parser.add_argument("--games", type=int, default=3)
    parser.add_argument("--only", default="")
    parser.add_argument("--n", type=int, default=0)
    parser.add_argument("--blind", action="store_true", help="plan with NOMINAL_GRIP, not the token's true place in the jaws")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--set", default="", help="scene parameters, e.g. riser=0.1,r0=0.12")
    parser.add_argument("--offset", type=float, default=0.0)
    parser.add_argument("--aim-mm", type=float, help="spike, stacks: the aim towards the fixed jaw (default 1)")
    args = parser.parse_args()
    if args.aim_mm is not None:
        _AIM.append(args.aim_mm / 1e3)
    result = COMMANDS[args.command](args)
    if args.json and result is not None:
        Path(args.json).write_text(json.dumps(result, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o)))


if __name__ == "__main__":
    sys.exit(main())
