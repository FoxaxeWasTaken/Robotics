import numpy as np, pinocchio as pin
URDF = "/ros2_ws/src/so101_description/urdf/so101.urdf"
full = pin.buildModelFromUrdf(URDF)
model = pin.buildReducedModel(full, [full.getJointId("gripper")], pin.neutral(full)); data = model.createData()
fid = model.getFrameId("gripper_frame_link"); lo, hi = model.lowerPositionLimit, model.upperPositionLimit
def fk(q): pin.framesForwardKinematics(model, data, q); return data.oMf[fid]
def ik5(p, a, q0, iters=300, dt=0.6, damp=1e-6):
    q = q0.copy()
    for _ in range(iters):
        T = fk(q); z = T.rotation[:, 2].copy()
        J6 = pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)
        Jr = -pin.skew(a) @ (-pin.skew(z) @ J6[3:])
        e = np.r_[p - T.translation, -np.cross(z, a)]
        Je = np.vstack([J6[:3], Jr])
        q = np.clip(pin.integrate(model, q, dt * Je.T @ np.linalg.solve(Je @ Je.T + damp * np.eye(6), e)), lo, hi)
    T = fk(q)
    return q, np.linalg.norm(T.translation - p), np.degrees(np.arccos(np.clip(T.rotation[:, 2] @ a, -1, 1)))
rng = np.random.default_rng(0)
seeds = [np.array([0, -0.3, 0.3, 1.2, 0]), np.array([0, 0.5, -0.5, 1.5, 0]), np.array([0, -1.0, 1.0, 0.5, 0])] + [rng.uniform(lo, hi) for _ in range(5)]
rs = [0.14, 0.18, 0.22, 0.26, 0.30, 0.34]
print("smallest |tilt| (deg) from vertical reachable exactly (pos<1mm, axis<1deg); + = tool tip toward outside; '-' none in [-80,80]")
print("z \\ r  " + "".join(f"{r:7.2f}" for r in rs))
for z in [0.02, 0.05, 0.08, 0.11, 0.14, 0.17, 0.20]:
    row = []
    for r in rs:
        found = None
        for th in sorted(range(-80, 81, 10), key=abs):
            t = np.radians(th); a = np.array([np.sin(t), 0, -np.cos(t)])
            if any(pe < 1e-3 and ae < 1 for _, pe, ae in (ik5(np.array([r, 0, z]), a, s0) for s0 in seeds[:4])):
                found = th; break
        row.append(f"{found:7d}" if found is not None else "      -")
    print(f"{z:4.2f}  " + "".join(row), flush=True)
