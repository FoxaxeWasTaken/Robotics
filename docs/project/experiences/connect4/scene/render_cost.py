"""OSMesa 320x240 frame cost of the game scene, per quality setting. LP_NUM_THREADS from the environment."""
import sys, os, time, tempfile
sys.path.insert(0, "/w")
import numpy as np, mujoco, cv2
from exp_connect4_scene import *

def rebuild(path, edit):
    spec = mujoco.MjSpec.from_file(str(path))
    edit(spec)
    out = path.with_name("edited.xml")
    out.write_text(spec.to_xml())
    return out

def frames(path, fill, label, n=40, save=None):
    m = mujoco.MjModel.from_xml_path(str(path)); d = mujoco.MjData(m)
    sim = Sim.__new__(Sim); sim.m, sim.d = m, d
    sim.p, sim.table_z = P, TZ
    sim.jq = {x: m.joint(x).qposadr[0] for x in ARM}; sim.act = {x: m.actuator(x).id for x in ARM}
    sim.home((1.5, -1.2, 1.2, 0.3, 0.0, 1.0))
    sim.fill(fill)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, height=240, width=320)
    r.update_scene(d, camera="external_cam"); img = r.render()
    ts = []
    for _ in range(n):
        t0 = time.perf_counter(); r.update_scene(d, camera="external_cam"); img = r.render(); ts.append(time.perf_counter() - t0)
    r.close()
    print(f"{label:34s} p50 {np.median(ts)*1e3:6.1f} ms  max {max(ts)*1e3:6.1f}  -> {1/np.median(ts):5.1f} Hz", flush=True)
    if save: cv2.imwrite(save, img[:, :, ::-1])

path, P, TZ = build(tempfile.mkdtemp(prefix="so101_"), **GAME_OPTIONS)
board = [2, 3, 6, 4, 1, 0, 5]
print("LP_NUM_THREADS", os.environ.get("LP_NUM_THREADS"))
frames(path, board, "game, as built", save="/w/render/cam_320.png")
def noshadow(s): s.visual.quality.shadowsize = 0
def norefl(s): s.material("groundplane").reflectance = 0
def nosamples(s): s.visual.quality.offsamples = 0
def noparked(s): pass
frames(rebuild(path, noshadow), board, "no shadows")
frames(rebuild(path, norefl), board, "no reflection")
frames(rebuild(path, nosamples), board, "no MSAA")
def all3(s): noshadow(s); norefl(s); nosamples(s)
frames(rebuild(path, all3), board, "no shadows, reflection, MSAA", save="/w/render/cam_320_fast.png")
def two(s): noshadow(s); norefl(s)
frames(rebuild(path, two), board, "no shadows, reflection")
