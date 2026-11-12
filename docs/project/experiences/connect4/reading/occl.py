"""Q2: which arm poses hide or show through the cells: geom segmentation, on the empty board (worst case: the
arm also shows through the holes). A cell counts as touched if a pixel of its sampling disc is arm or held token."""
import numpy as np, cv2, sys
from common import *

def touched(w, g, T_cam, hold=False):
    seg = w.seg(); bad = np.isin(seg, list(w.arm_geoms))
    if hold:
        gid = w.m.body_geomadr[w.m.body("tok_y21").id]; bad |= seg == gid
    uv, _ = project(np.array([[c * PX, TOKEN_T / 2, r * g.pz] for r in range(ROWS) for c in range(COLS)]), g.T, T_cam, w.K)
    rad = 0.3 * PX / np.linalg.norm(T_cam[:3, 3] - g.t) * w.K[0, 0]
    masks = disc_masks(uv, rad, seg.shape)
    cells = sum(bad[m].any() for m in masks)
    mk = 0
    for i in g.markers():
        c = project(g.marker_corners(i), g.T, T_cam, w.K)[0].astype(int)
        p = cv2.fillConvexPoly(np.zeros(seg.shape, np.uint8), c, 1) > 0
        mk += bad[p].any()
    return cells, mk

def hold_token(w):
    """A yellow token between the open jaws (at the TCP site), its axis along the jaw closing axis."""
    mujoco.mj_forward(w.m, w.d)
    a = w.tok_adr["y21"]; s = w.d.site("gripperframe")
    w.d.qpos[a:a + 3] = s.xpos; R = s.xmat.reshape(3, 3)
    w.d.qpos[a + 3:a + 7] = mat2quat(R)

if __name__ == "__main__":
  for az in (-51, -40):
      g = Grid(az=az, marker=0.03)
      w = World(g, 640, 480, cam=P1_CAM); T = w.T_true()
      print(f"== grid az {az}: pan (rad, az = -pan) -> cells touched / markers touched, for arm shapes (TCP in base_link at that pan)")
      for name, shape in (("mid   ", [-0.6, 0.6, 1.1, 1.57]), ("low   ", [0.0, 0.4, 1.3, 1.57]), ("up    ", [-1.0, 0.8, 0.5, 1.57]), ("fold  ", [-1.7, 1.5, 0.7, 0.0])):
          out = []
          for pan in np.arange(-1.9, 1.91, 0.2):
              w.arm([pan, *shape]); hold_token(w)
              c, m = touched(w, g, T, hold=True)
              out.append(f"{pan:+.1f}:{c}/{m}")
          w.arm([-1.0, *shape]); mujoco.mj_forward(w.m, w.d)
          print(f"  {name} tcp@pan-1.0 {np.round(w.d.site('gripperframe').xpos, 3)}  " + " ".join(out))
      w.close()
