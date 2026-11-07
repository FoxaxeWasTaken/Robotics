"""Shared bench: the Connect Four model on SO101Sim in-process, with the driver's tick and the brain's min-jerk.
A scratch model of the grid (solid walls, ribs), not the real scene (P2.3a)."""
import shutil, tempfile
from pathlib import Path
import mujoco, numpy as np
from so101_sim import SO101Sim, DEFAULT_SCENE
from so101_brain.kinematics import Kinematics, PAN_AXIS_X
from so101_brain.trajectory import min_jerk
from so101_driver.limiter import step_towards
URDF = "/repo/ros2_ws/src/so101_description/urdf/so101.urdf"
ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
DT = 0.02
V_MAX, T_MIN, REACH_TOL, MARGIN = 1.0, 0.5, 0.01, 1.0      # brain.yaml motion.*
MAX_JOINT_V, MAX_GRIP_V = 1.5, 200.0                       # driver.yaml
RISER = 0.08
PITCH, COLS, ROWS = 0.037, 7, 6
R0 = 0.14
TOP = 0.25                                                  # grid top above the table (world z)
WALL = 0.003                                                # one face thickness (model)
RIB = 0.0015                                                # column divider thickness (model)
TOKEN_R, TOKEN_T = 0.017, 0.0065

def build(gap=0.0095, az=0.0, offset=0.0, tokens=2, grid=True, guide=False, token_mass=0.003, token_friction=None, solref=None, solimp=None, condim=None, multiccd=False, margin=None):
    folder = Path(tempfile.mkdtemp(prefix="c4_")) / "so101"
    shutil.copytree(Path(DEFAULT_SCENE).parent, folder)
    spec = mujoco.MjSpec.from_file(str(folder / Path(DEFAULT_SCENE).name))
    spec.body("base").pos[2] += RISER
    u = np.array([np.cos(az), np.sin(az), 0.0]); n = np.array([-np.sin(az), np.cos(az), 0.0])
    o = np.array([PAN_AXIS_X, 0, 0]) + offset * n
    yaw = np.array([np.cos(az / 2), 0, 0, np.sin(az / 2)])
    b = spec.worldbody
    def box(name, r, across, z, half):
        b.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=half, pos=o + r * u + across * n + [0, 0, z], quat=yaw,
                   rgba=[0.1, 0.2, 0.8, 1])
    if grid:
        L = COLS * PITCH + RIB; H = ROWS * PITCH; rc = R0 + 3 * PITCH; zc = TOP - H / 2
        for s, name in ((1, "wall_p"), (-1, "wall_m")):
            box(name, rc, s * (gap / 2 + WALL / 2), zc, [L / 2, WALL / 2, H / 2])
        for i in range(COLS + 1):
            box(f"rib_{i}", R0 + (i - 0.5) * PITCH, 0, zc, [RIB / 2, gap / 2, H / 2])
        box("grid_floor", rc, 0, TOP - H - 0.002, [L / 2, gap / 2, 0.002])
    if guide:  # a V across the grid, 20 mm entry down to the gap, 15 mm high
        for s in (1, -1):
            g = spec.worldbody.add_geom(name=f"guide_{'p' if s > 0 else 'm'}", type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[COLS * PITCH / 2, 0.0015, 0.0092],
                pos=o + (R0 + 3 * PITCH) * u + s * (gap / 2 + 0.00456) * n + [0, 0, TOP + 0.0075], rgba=[0.3, 0.3, 0.9, 1])
            ang = -s * np.arctan2(0.0055, 0.015)
            qt = np.zeros(4); mujoco.mju_axisAngle2Quat(qt, u, ang)
            qq = np.zeros(4); mujoco.mju_mulQuat(qq, qt, yaw); g.quat = qq
    for i in range(tokens):
        t = spec.worldbody.add_body(name=f"token_{i}", pos=[1.0 + 0.05 * i, 1.0, 0.02])
        t.add_freejoint(name=f"token_{i}_free")
        g = t.add_geom(name=f"token_{i}_geom", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[TOKEN_R, TOKEN_T / 2, 0], mass=token_mass,
                       rgba=[0.95, 0.85, 0.1, 1])
        if token_friction is not None:
            g.friction = token_friction
        g.priority = 1
        if solref is not None:
            g.solref = solref
        if solimp is not None:
            g.solimp = solimp
        if condim is not None:
            g.condim = condim
        if margin is not None:
            g.margin = margin
    if multiccd:
        spec.option.enableflags |= mujoco.mjtEnableBit.mjENBL_MULTICCD
    path = folder / "scene_c4.xml"
    path.write_text(spec.to_xml())
    return str(path)

class Bench:
    def __init__(self, seed=0, **kw):
        self.kw = kw
        self.sim = SO101Sim(scene_path=build(**kw), seed=seed)
        self.sim.connect()
        m, d = self.sim.model, self.sim.data
        self.m, self.d = m, d
        a = m.joint("pickup_object_free").qposadr[0]
        d.qpos[a:a + 3] = [-1.0, -1.0, 0.03]
        m.body_pos[m.body("drop_box").id, :2] = (2.0, 2.0)
        mujoco.mj_forward(m, d)
        self.setpoint = {n: 0.0 for n in ARM} | {"gripper": 100.0}
        self.goal = dict(self.setpoint)
        self.obs = self.sim.get_observation()
        self.hand = None      # (qadr, pose7): the sim's hand holds the token still
        self.on_tick = None
        self.t = 0.0
    def q(self):
        return np.array([self.obs[f"{n}.pos"] for n in ARM])
    def teleport_arm(self, q, grip=100.0):
        for n, v in zip(ARM, q):
            self.d.qpos[self.m.joint(n).qposadr[0]] = v
            self.setpoint[n] = self.goal[n] = float(v)
        g = self.m.joint("gripper").range
        self.d.qpos[self.m.joint("gripper").qposadr[0]] = g[0] + grip / 100 * (g[1] - g[0])
        self.setpoint["gripper"] = self.goal["gripper"] = grip
        self.d.ctrl[:] = 0
        for i, n in enumerate((*ARM, "gripper")):
            self.d.ctrl[self.m.actuator(n).id] = self.setpoint[n] if n != "gripper" else self.d.qpos[self.m.joint("gripper").qposadr[0]]
        self.d.qvel[:] = 0
        mujoco.mj_forward(self.m, self.d)
        self.obs = self.sim.get_observation()
    def tick(self):
        self.setpoint = step_towards(self.setpoint, self.goal, DT, MAX_JOINT_V, MAX_GRIP_V)
        self.sim.send_action({f"{n}.pos": v for n, v in self.setpoint.items()})
        if self.hand:
            a, v, pose = self.hand
            self.d.qpos[a:a + 7] = pose; self.d.qvel[v:v + 6] = 0
        self.obs = self.sim.get_observation()
        if self.hand:
            a, v, pose = self.hand
            self.d.qpos[a:a + 7] = pose; self.d.qvel[v:v + 6] = 0
        self.t += DT
        if self.on_tick:
            self.on_tick(self)
    def move(self, q_goal, v_max=V_MAX):
        tr = min_jerk(self.q(), q_goal, v_max, T_MIN)
        t = 0.0
        while True:
            self.goal.update(zip(ARM, tr.sample(t)))
            self.tick()
            t += DT
            if t >= tr.duration and np.max(np.abs(self.q() - q_goal)) < REACH_TOL:
                return t, tr.duration
            if t > tr.duration + MARGIN:
                return None, tr.duration
    def wait(self, s):
        for _ in range(round(s / DT)):
            self.tick()
    def gripper(self, pct, s):
        self.goal["gripper"] = pct
        self.wait(s)
    def effort(self):
        a = self.m.actuator("gripper").id
        return float(-self.d.actuator_force[a] / self.m.actuator_forcerange[a][1])
    def tcp(self):
        s = self.d.site("gripperframe")
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()   # NB site rotated 90 deg vs the URDF TCP
    def token_pose(self, i=0):
        b = self.d.body(f"token_{i}")
        return b.xpos.copy(), b.xmat.reshape(3, 3).copy()
    def token_adr(self, i=0):
        j = self.m.joint(f"token_{i}_free")
        return j.qposadr[0], j.dofadr[0]
    def contacts(self, a=None):
        out = set()
        for c in self.d.contact[: self.d.ncon]:
            g1, g2 = self.m.geom(c.geom1), self.m.geom(c.geom2)
            out.add((self.m.body(self.m.geom_bodyid[c.geom1]).name, g1.name, self.m.body(self.m.geom_bodyid[c.geom2]).name, g2.name, round(float(c.dist) * 1e3, 2)))
        return out
    def set_token(self, pos, R, i=0):
        q = np.zeros(4); mujoco.mju_mat2Quat(q, R.flatten())
        a, v = self.token_adr(i)
        self.d.qpos[a:a + 3] = pos; self.d.qpos[a + 3:a + 7] = q; self.d.qvel[v:v + 6] = 0
        mujoco.mj_forward(self.m, self.d)
        return np.r_[pos, q]

def fk_world(kin, q):
    T = kin.fk(q).copy(); T[2, 3] += RISER
    return T
