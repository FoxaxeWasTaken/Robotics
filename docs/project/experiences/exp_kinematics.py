"""Expert checks: sim timing, URDF<->MJCF FK consistency, tool frame axes, shoulder_pan sign."""
import time
import numpy as np
import mujoco
import pinocchio as pin
from so101_sim import SO101Sim, ARM_JOINTS

URDF = "/ros2_ws/src/so101_description/urdf/so101.urdf"
np.set_printoptions(precision=4, suppress=True)

sim = SO101Sim(seed=1)
sim.connect()
m, d = sim.model, sim.data
print(f"[timing] timestep={m.opt.timestep} s, steps/call={sim.sim_dt_per_step} -> {m.opt.timestep*sim.sim_dt_per_step*1e3:.1f} ms sim per get_observation/send_action")

# ---------- Pinocchio model (gripper locked) ----------
full = pin.buildModelFromUrdf(URDF)
print("[pin] joints:", [full.names[i] for i in range(1, full.njoints)])
model = pin.buildReducedModel(full, [full.getJointId("gripper")], pin.neutral(full))
data = model.createData()
fid = model.getFrameId("gripper_frame_link")
names = [model.names[i] for i in range(1, model.njoints)]
assert names == ARM_JOINTS, names
lo, hi = model.lowerPositionLimit, model.upperPositionLimit

def fk(q):
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[fid].copy()

site = m.site("gripperframe").id
qadr = [m.joint(n).qposadr[0] for n in ARM_JOINTS]

def mj_site(q):
    d.qpos[qadr] = q
    mujoco.mj_kinematics(m, d)
    return d.site_xpos[site].copy(), d.site_xmat[site].reshape(3, 3).copy()

# ---------- FK consistency ----------
rng = np.random.default_rng(0)
errs, rerrs = [], []
for _ in range(500):
    q = rng.uniform(lo, hi)
    T = fk(q)
    p, R = mj_site(q)
    errs.append(np.linalg.norm(T.translation - p))
    rerrs.append(np.linalg.norm(pin.log3(T.rotation.T @ R)))
print(f"[fk] URDF gripper_frame_link vs MJCF site gripperframe over 500 q: pos err max={max(errs)*1e3:.3f} mm, rot err max={np.degrees(max(rerrs)):.3f} deg")

# which gripper-frame axis is the approach axis (along wrist_roll axis, pointing out of the tool)?
q0 = np.zeros(5)
T0 = fk(q0)
pin.forwardKinematics(model, data, q0)
wr = data.oMi[model.getJointId("wrist_roll")]
toolward = T0.translation - wr.translation
for i, a in enumerate("xyz"):
    print(f"[frame] gripper_frame {a}-axis . (wrist_roll->tcp dir) = {T0.rotation[:, i] @ toolward / np.linalg.norm(toolward):+.3f}")


pin.forwardKinematics(model, data, q0)
print("[convention] shoulder_pan axis xy =", data.oMi[model.getJointId("shoulder_pan")].translation[:2])
print("[convention] TCP at q=0:", fk(q0).translation, "| pan=+0.5:", fk(np.r_[0.5, 0, 0, 0, 0]).translation, "(pan>0 -> y<0)")
