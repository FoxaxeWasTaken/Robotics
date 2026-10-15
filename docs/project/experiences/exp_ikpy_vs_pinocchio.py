"""Cross-check: ikpy (team's current choice) vs Pinocchio DLS on the same targets."""
import time, warnings, numpy as np, pinocchio as pin
warnings.filterwarnings("ignore")
from ikpy.chain import Chain
URDF = "/ros2_ws/src/so101_description/urdf/so101.urdf"
chain = Chain.from_urdf_file(URDF, base_elements=["base_link"], active_links_mask=[False] + [True] * 5 + [False])
full = pin.buildModelFromUrdf(URDF)
model = pin.buildReducedModel(full, [full.getJointId("gripper")], pin.neutral(full)); data = model.createData()
fid = model.getFrameId("gripper_frame_link"); lo, hi = model.lowerPositionLimit, model.upperPositionLimit
def fk(q): pin.framesForwardKinematics(model, data, q); return data.oMf[fid]
def dls(q, J, e): return np.clip(pin.integrate(model, q, 0.5 * J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(len(e)), e)), lo, hi)
def pin_pos(p, q0):
    q = q0.copy()
    for _ in range(300):
        e = p - fk(q).translation
        if np.linalg.norm(e) < 2e-4: break
        q = dls(q, pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)[:3], e)
    return q
def pin_point(p, a, q0):
    q = q0.copy()
    for _ in range(300):
        T = fk(q); z = T.rotation[:, 2].copy(); e = np.r_[p - T.translation, -np.cross(z, a)]
        if np.linalg.norm(e[:3]) < 2e-4 and np.linalg.norm(e[3:]) < 2e-3: break
        J6 = pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL_WORLD_ALIGNED)
        q = dls(q, np.vstack([J6[:3], -pin.skew(a) @ (-pin.skew(z) @ J6[3:])]), e)
    return q
def errs(q, p, a=None):
    T = fk(q); pe = np.linalg.norm(T.translation - p)
    ae = 0 if a is None else np.degrees(np.arccos(np.clip(T.rotation[:, 2] @ a, -1, 1)))
    return pe, ae
rng = np.random.default_rng(7); res = {k: [] for k in ["ikpy_pos", "ikpy_pos_zero", "pin_pos", "ikpy_point", "pin_point"]}; tm = {k: [] for k in res}
for _ in range(100):
    r, ang = rng.uniform(0.18, 0.30), np.radians(rng.uniform(-50, 60))
    p = np.array([r * np.cos(ang), r * np.sin(ang), rng.uniform(0.02, 0.06)]); a = np.array([0, 0, -1.0])
    az = np.arctan2(p[1], p[0] - 0.0388); seed = np.array([-az, -0.3, 0.3, 1.2, 0.0]); zero = np.zeros(5)
    for key, fn in [("ikpy_pos", lambda: chain.inverse_kinematics(p, initial_position=np.r_[0, seed, 0])[1:6]),
                    ("ikpy_pos_zero", lambda: chain.inverse_kinematics(p, initial_position=np.r_[0, zero, 0])[1:6]),
                    ("pin_pos", lambda: pin_pos(p, seed)),
                    ("ikpy_point", lambda: chain.inverse_kinematics(p, a, orientation_mode="Z", initial_position=np.r_[0, seed, 0])[1:6]),
                    ("pin_point", lambda: pin_point(p, a, seed))]:
        t = time.perf_counter(); q = fn(); tm[key].append(time.perf_counter() - t)
        res[key].append(errs(q, p, a if "point" in key else None))
for k in res:
    e = np.array(res[k]); ok = (e[:, 0] < 1e-3) & (e[:, 1] < 2)
    print(f"{k:14s} success(<1mm{', <2deg' if 'point' in k else ''}) {ok.mean()*100:5.1f}% | median pos err {np.median(e[:,0])*1e3:6.2f} mm | p95 {np.percentile(e[:,0],95)*1e3:6.2f} mm | median time {np.median(tm[k])*1e3:6.1f} ms")

# MES-07: position-only IK over the full height range used by lift / transport (z 2-12 cm)
ok_ikpy, ok_pin = [], []
for _ in range(200):
    r, ang = rng.uniform(0.18, 0.30), np.radians(rng.uniform(-50, 60))
    p = np.array([r * np.cos(ang), r * np.sin(ang), rng.uniform(0.02, 0.12)])
    seed = np.array([-np.arctan2(p[1], p[0] - 0.0388), -0.3, 0.3, 1.2, 0.0])
    ok_ikpy.append(errs(chain.inverse_kinematics(p, initial_position=np.r_[0, seed, 0])[1:6], p)[0] < 1e-3)
    ok_pin.append(errs(pin_pos(p, seed), p)[0] < 1e-3)
print(f"[MES-07] position-only, z 2-12 cm, 200 targets: ikpy {np.mean(ok_ikpy)*100:.1f}% | pinocchio {np.mean(ok_pin)*100:.1f}%")
