"""MES-54: does the sim calibration view render the same on every machine (#187)? The view of
test_sim_scene.py's calibration tests (our scene with the ArUco markers, 640x480, the arm in PARK), rendered
by OSMesa (llvmpipe), then the calibration's detection and solve: the image's hash, the mean reprojection
error against the gate of markers_sim.yaml (1.5 px since #217), and the worst detected corner against its projection.

llvmpipe compiles its shaders for the host CPU: a runner with another instruction set may render other
pixels. Run it in one process per llvmpipe setting, e.g. LP_NATIVE_VECTOR_WIDTH=128 (what a CPU without
AVX gets) or LP_NUM_THREADS=1, and compare the lines. Then with N, the same view for the sim seeds 0 to N-1:
SO101Sim.connect() draws the cube's pose, and the test takes no seed. In the control image, the workspace built
and sourced:
    python3 exp_sim_calibration_render.py [N]
"""

import hashlib
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from so101_sim import ARM_JOINTS, SO101Sim

from so101_bringup import sim_scene
from so101_common.markers import marker_corners
from so101_driver.camera import opencv_intrinsics
from so101_perception import calibration

PARK = [1.9, -1.7, 1.5, 0.7, 0.0]  # brain.yaml's fsm.park_pose, where the calibration is done
FLIP = np.diag([1.0, -1.0, -1.0])
CONFIG = Path(get_package_share_directory("so101_bringup")) / "config"
LAYOUT = yaml.safe_load((CONFIG / "markers_sim.yaml").read_text())
CAMERA = yaml.safe_load((CONFIG / "sim_scene.yaml").read_text())["camera"]

cpu = subprocess.run(["lscpu"], capture_output=True, text=True).stdout
model = next((line.split(":", 1)[1].strip() for line in cpu.splitlines() if line.startswith("Model name")), "?")
flags = next((line.split(":", 1)[1].split() for line in cpu.splitlines() if line.startswith("Flags")), [])
isa = "avx512f" if "avx512f" in flags else "avx2" if "avx2" in flags else "sse"
setting = {k: v for k, v in os.environ.items() if k.startswith(("LP_", "GALLIVM_"))}

scene = sim_scene.generate(tempfile.mkdtemp(), camera=CAMERA, markers=LAYOUT)


def view(seed):
    """The view of the test for that sim seed (connect() draws the cube's pose): the image, the markers found,
    the solve ("refused" or its error, px), the worst corner offset (px, its marker) and the cube's xy."""
    sim = SO101Sim(scene_path=scene, camera_width=640, camera_height=480, seed=seed)
    sim.connect()
    for name, q in zip(ARM_JOINTS, PARK, strict=True):
        sim.data.qpos[sim.model.joint(name).qposadr[0]] = q
    mujoco.mj_forward(sim.model, sim.data)
    renderer = mujoco.Renderer(sim.model, height=480, width=640)
    renderer.update_scene(sim.data, camera=sim.camera_name)
    image = renderer.render()
    renderer.close()
    found = calibration.detect_markers(image, LAYOUT["dictionary"])
    K = opencv_intrinsics(sim.get_camera_intrinsics())
    T = sim.get_camera_extrinsics()  # spike only: the rendered camera, to score the corners
    R, t = T[:3, :3] @ FLIP, T[:3, 3]
    offsets = {}
    for i, detected in found.items():
        camera = (marker_corners(LAYOUT["markers"][i], LAYOUT["side_m"]) - t) @ R
        uv = (camera @ K.T)[:, :2] / camera[:, 2:]
        offsets[i] = float(np.abs(uv - detected).max())
    try:
        error = calibration.solve([found], LAYOUT, K)[1]
    except calibration.CalibrationError:
        error = None
    worst = max(offsets, key=offsets.get)
    cube = sim.data.body("pickup_object").xpos[:2].round(3).tolist()
    sim.disconnect()
    return image, found, error, (offsets[worst], worst), cube


image, found, error, (offset, worst), _ = view(None)  # as the test: no seed
print(
    f"{model} ({isa}, {os.cpu_count()} CPUs) {setting or 'default llvmpipe'}, no seed: image "
    f"{hashlib.sha256(image.tobytes()).hexdigest()[:12]}, markers {sorted(found)}, reprojection "
    f"{'refused' if error is None else f'{error:.3f} px'}, worst corner marker {worst} {offset:.2f} px",
    flush=True,
)
SEEDS = int(sys.argv[1]) if len(sys.argv) > 1 else 0
bad = []
for seed in range(SEEDS):
    _, found, error, (offset, worst), cube = view(seed)
    if error is None or error >= LAYOUT["max_reprojection_px"] or offset >= 8.0 or len(found) < len(LAYOUT["markers"]):
        bad.append((seed, cube, sorted(found), error and round(error, 3), worst, round(offset, 2)))
if SEEDS:
    print(f"{len(bad)}/{SEEDS} seeds fail the tests' bounds (reprojection < 1 px, every corner within 5 px):")
    for row in bad:
        print("   seed {}: cube at {}, markers {}, reprojection {} px, worst corner marker {} {} px".format(*row))
