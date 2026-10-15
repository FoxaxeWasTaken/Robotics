"""Feasibility spike v2: scripted pick & place with ground truth (test only)."""
import numpy as np, mujoco, pinocchio as pin
from so101_sim import SO101Sim, ARM_JOINTS

full = pin.buildModelFromUrdf("/ros2_ws/src/so101_description/urdf/so101.urdf")
model = pin.buildReducedModel(full, [full.getJointId("gripper")], pin.neutral(full))
data = model.createData()
fid = model.getFrameId("gripper_frame_link")
lo, hi = model.lowerPositionLimit, model.upperPositionLimit

def fk(q):
    pin.framesForwardKinematics(model, data, q); return data.oMf[fid].copy()

def dls(q, J, e, dt=0.5, damp=1e-6):
    return np.clip(pin.integrate(model, q, dt * J.T @ np.linalg.solve(J @ J.T + damp * np.eye(len(e)), e)), lo, hi)

def ik_pose(p, R, q0, iters=400):
    q = q0.copy(); M = pin.SE3(R, p)
    for _ in range(iters):
        e = pin.log6(fk(q).actInv(M)).vector
        if np.linalg.norm(e[:3]) < 2e-4 and np.linalg.norm(e[3:]) < 2e-3: return q
        q = dls(q, pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL), e)
    return None

def ik_pos(p, q0, iters=300):
    q = q0.copy()
    for _ in range(iters):
        e = p - fk(q).translation
        if np.linalg.norm(e) < 2e-4: return q
        q = dls(q, pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)[:3], e)
    return None

print("[convention] TCP xyz at q=0:", np.round(fk(np.zeros(5)).translation, 3),
      "| pan=+0.5:", np.round(fk(np.r_[0.5, 0, 0, 0, 0]).translation, 3))

sim = SO101Sim(seed=0); sim.connect(); m, d = sim.model, sim.data
qadr = [m.joint(n).qposadr[0] for n in ARM_JOINTS]
d.qpos[qadr] = 0; mujoco.mj_forward(m, d)
T0 = fk(np.zeros(5))
c_local = T0.rotation.T @ np.cross(d.xaxis[m.joint("gripper").id], T0.rotation[:, 2]); c_local /= np.linalg.norm(c_local)

def R_target(a, closing):
    closing = closing - a * (closing @ a); closing /= np.linalg.norm(closing)
    W = np.column_stack([closing, np.cross(a, closing), a])
    zl = np.array([0, 0, 1.0]); L = np.column_stack([c_local, np.cross(zl, c_local), zl])
    return W @ L.T

pin.forwardKinematics(model, data, np.zeros(5))
PAN_AXIS_XY = data.oMi[model.getJointId("shoulder_pan")].translation[:2].copy()
print("[convention] shoulder_pan axis at xy =", np.round(PAN_AXIS_XY, 4))
def arm_az(p):  # azimuth of the arm's vertical plane = seen from the pan axis, not from base origin
    return np.arctan2(p[1] - PAN_AXIS_XY[1], p[0] - PAN_AXIS_XY[0])
SEEDS = [np.array([0, -0.3, 0.3, 1.2, 0]), np.array([0, 0.5, -0.5, 1.5, 0]), np.array([0, -1.0, 1.0, 0.5, 0])]
def seeds_for(p):
    pan = -arm_az(p)            # shoulder_pan is reversed w.r.t. world yaw
    return [s + np.r_[pan, 0, 0, 0, 0] for s in SEEDS]

def plan_grasp(p, yaw, backoff=0.04):
    """Minimal-tilt grasp aligned with cube faces + pre-grasp backed off along the approach axis."""
    az = arm_az(p)
    for tilt in np.radians([0, 10, 20, 30]):
        a = np.array([np.sin(tilt) * np.cos(az), np.sin(tilt) * np.sin(az), -np.cos(tilt)])
        for k in range(4):
            R = R_target(a, np.array([np.cos(yaw + k * np.pi / 2), np.sin(yaw + k * np.pi / 2), 0]))
            for s0 in seeds_for(p):
                qg = ik_pose(p, R, s0)
                if qg is None: continue
                qp = ik_pose(p - backoff * a, R, qg)
                if qp is not None: return qg, qp, np.degrees(tilt)
    return None, None, None

def ik_point(p, a, q0, roll=None, iters=400):
    """position + approach axis; if roll is given, wrist_roll is frozen at that value."""
    q = q0.copy()
    if roll is not None: q[4] = roll
    for _ in range(iters):
        T = fk(q); z = T.rotation[:, 2].copy()
        e = np.r_[p - T.translation, -np.cross(z, a)]
        if np.linalg.norm(e[:3]) < 2e-4 and np.linalg.norm(e[3:]) < 2e-3: return q
        J6 = pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)
        J = np.vstack([J6[:3], -pin.skew(a) @ (-pin.skew(z) @ J6[3:])])
        if roll is not None: J[:, 4] = 0
        q = dls(q, J, e)
    return None

def jaw_yaw(q):
    T = fk(q); c = T.rotation @ c_local; return np.arctan2(c[1], c[0])

def plan_grasp_decoupled(p, yaw, backoff=0.04):
    az = arm_az(p)
    for tilt in np.radians([0, 10, 20, 30]):
        a = np.array([np.sin(tilt) * np.cos(az), np.sin(tilt) * np.sin(az), -np.cos(tilt)])
        for s0 in seeds_for(p):
            q = ik_point(p, a, s0)                        # 1) reach + point, roll free
            if q is None: continue
            # 2) choose roll so that the jaw axis matches a cube face normal (90 deg symmetry), within limits
            err = (yaw - jaw_yaw(q) + np.pi / 4) % (np.pi / 2) - np.pi / 4
            for roll in sorted([q[4] + err + k * np.pi / 2 for k in range(-2, 3)], key=lambda r: abs(r - q[4])):
                if not (lo[4] <= roll <= hi[4]): continue
                # sign of roll effect on jaw yaw may be reversed: try both
                for rr in (roll, 2 * q[4] - roll):
                    if not (lo[4] <= rr <= hi[4]): continue
                    qg = ik_point(p, a, q, roll=rr)       # 3) re-solve position with roll frozen
                    if qg is None: continue
                    e_yaw = (yaw - jaw_yaw(qg) + np.pi / 4) % (np.pi / 2) - np.pi / 4
                    if abs(e_yaw) > np.radians(3): continue
                    qp = ik_point(p - backoff * a, a, qg, roll=rr)
                    if qp is not None: return qg, qp, np.degrees(tilt)
    return None, None, None

def plan_grasp_free(p, backoff=0.04):
    """No jaw/face alignment: position + approach axis only (roll free)."""
    az = arm_az(p)
    for tilt in np.radians([0, 10, 20, 30, 40]):
        a = np.array([np.sin(tilt) * np.cos(az), np.sin(tilt) * np.sin(az), -np.cos(tilt)])
        for s0 in seeds_for(p):
            qg = ik_point(p, a, s0)
            if qg is None: continue
            qp = ik_point(p - backoff * a, a, qg, roll=qg[4])
            if qp is not None: return qg, qp, np.degrees(tilt)
    return None, None, None

def plan_pos(p, q_hint):
    for s0 in [q_hint] + seeds_for(p):
        q = ik_pos(p, s0)
        if q is not None: return q
    return None

site = m.site("gripperframe").id
def move(q_goal, grip, T=1.5):
    obs = sim.get_observation(); q0 = np.array([obs[f"{n}.pos"] for n in ARM_JOINTS])
    n = int(T / 0.01)
    for i in range(1, n + 1):
        s = i / n; s = 10 * s**3 - 15 * s**4 + 6 * s**5      # minimum-jerk
        sim.send_action({**{f"{j}.pos": float(v) for j, v in zip(ARM_JOINTS, q0 + s * (q_goal - q0))}, "gripper.pos": grip})
    for _ in range(30): sim.get_observation()

ok_count, rows = 0, []
for seed in range(40):
    sim._rng = np.random.default_rng(100 + seed)
    d.qpos[qadr] = 0; d.ctrl[:] = 0; sim.randomize_object()
    obj = sim.get_object_position()                               # GT, test only
    qa = m.joint("pickup_object_free").qposadr[0]; yaw = 2 * np.arctan2(d.qpos[qa + 6], d.qpos[qa + 3])
    box = sim.get_drop_box_position()
    qg, qp, tilt = plan_grasp_decoupled(obj, yaw); mode = "aligned"
    if qg is None:
        qg, qp, tilt = plan_grasp_free(obj); mode = "free"
    if qg is None: rows.append((seed, "IK-FAIL-grasp", np.round(obj, 3))); continue
    q_lift = plan_pos(obj + [0, 0, 0.10], qp)
    q_box = plan_pos(box + [0, 0, 0.11], q_lift if q_lift is not None else qp)
    if q_lift is None or q_box is None: rows.append((seed, "IK-FAIL-transport")); continue
    move(qp, 100.0); move(qg, 100.0, T=1.0)
    tcp_err = np.linalg.norm(d.site_xpos[site] - obj)
    move(qg, 0.0, T=0.8)
    grip = sim.get_observation()["gripper.pos"]; eff = float(d.actuator_force[m.actuator("gripper").id])
    move(qp, 0.0, T=0.8); move(q_lift, 0.0, T=1.0)
    lifted = sim.get_object_position()[2] - obj[2]
    move(q_box, 0.0, T=2.0); move(q_box, 100.0, T=0.8)
    for _ in range(150): sim.get_observation()
    o = sim.get_object_position()
    in_box = bool(abs(o[0] - box[0]) < 0.05 and abs(o[1] - box[1]) < 0.05 and o[2] < 0.06)
    ok_count += in_box
    rows.append((seed, "OK" if in_box else "MISS", f"r={np.hypot(*obj[:2]):.2f}", mode, f"tilt={tilt:.0f}", f"jaw_vs_face={np.degrees((yaw - jaw_yaw(qg) + np.pi/4) % (np.pi/2) - np.pi/4):+.0f}deg", f"tcp_err={tcp_err*1e3:.1f}mm",
                 f"grip%={grip:.1f}", f"grip_effort={eff:+.2f}Nm", f"lift={lifted*100:.1f}cm"))
for r in rows: print("[pick]", r)
print(f"[pick] success {ok_count}/{len(rows)}")
move(q_lift, 0.0, T=1.0)
print("[pick] closed on nothing: gripper %", round(sim.get_observation()["gripper.pos"], 1), "effort", round(float(d.actuator_force[m.actuator("gripper").id]), 2), "Nm")
