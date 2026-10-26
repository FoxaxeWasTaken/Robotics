"""sim_scene - our sim scenes: a copy of the provided scene edited with mujoco.MjSpec, never sim/ (ADR-036).

generate() copies the provided scene folder (its include and mesh paths resolve from the main file), edits the
copy and writes the result next to it; the driver loads it through its scene_path parameter. SimScene is the
launch substitution that does it at launch time, in sim only.

- camera: the pose of the external camera, config/sim_scene.yaml (#47): the provided one cut 21 % of the
  cubes at the image edge [MES-12];
- markers: the ArUco markers of config/markers_sim.yaml on the table, for the extrinsic calibration
  (3.4a, #44).
"""

import shutil
import tempfile
from pathlib import Path

import cv2
import mujoco
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitution import Substitution
from launch.utilities import perform_substitutions
from so101_sim import DEFAULT_SCENE

from so101_common.markers import marker_corners  # noqa: F401 (the convention the textures follow)

MARKER_PIXELS = 600  # texture resolution of a marker's black square


def generate(folder, camera=None, markers=None) -> Path:
    """Our scene in folder (a copy of the provided one, edited); returns the path of its main file."""
    folder = Path(folder) / "so101"
    shutil.copytree(Path(DEFAULT_SCENE).parent, folder)
    spec = mujoco.MjSpec.from_file(str(folder / Path(DEFAULT_SCENE).name))
    if camera:
        cam = spec.camera("external_cam")
        cam.pos = camera["pos"]
        cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES  # the provided camera uses xyaxes, which
        cam.alt.xyaxes = camera["xyaxes"]  # a quat set alone would not override
    if markers:
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, markers["dictionary"]))
        border = round(MARKER_PIXELS * (markers["plane_m"] - markers["side_m"]) / markers["side_m"] / 2)
        half = markers["plane_m"] / 2
        for i, (x, y) in markers["markers"].items():
            image = cv2.aruco.generateImageMarker(dictionary, i, MARKER_PIXELS)
            image = cv2.copyMakeBorder(image, *(border,) * 4, cv2.BORDER_CONSTANT, value=255)  # quiet zone
            cv2.imwrite(str(folder / f"aruco_{i}.png"), np.rot90(image, -1))  # see so101_common.markers
            spec.add_texture(name=f"aruco_{i}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=f"aruco_{i}.png")
            material = spec.add_material(name=f"aruco_{i}")
            material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = f"aruco_{i}"
            spec.worldbody.add_geom(
                name=f"aruco_{i}",
                type=mujoco.mjtGeom.mjGEOM_PLANE,
                size=[half, half, 0.001],
                pos=[x, y, 0.0005],  # just above the floor: no z-fighting
                material=f"aruco_{i}",
                contype=0,
                conaffinity=0,
            )
    path = folder / "scene_so101.xml"
    path.write_text(spec.to_xml())
    return path


class SimScene(Substitution):
    """The path of our scene, generated at launch: in sim, the moved camera, and the markers with calibration;
    empty (the provided scene) on the real arm."""

    def __init__(self, use_sim, calibration):
        super().__init__()
        self._use_sim, self._calibration = use_sim, calibration

    def describe(self):
        return "SimScene(use_sim, calibration)"

    def perform(self, context):
        if perform_substitutions(context, [self._use_sim]).lower() != "true":
            return ""
        config = Path(get_package_share_directory("so101_bringup")) / "config"
        camera = yaml.safe_load((config / "sim_scene.yaml").read_text())["camera"]
        markers = None
        if perform_substitutions(context, [self._calibration]).lower() == "true":
            markers = yaml.safe_load((config / "markers_sim.yaml").read_text())
        return str(generate(tempfile.mkdtemp(prefix="so101_scene_"), camera=camera, markers=markers))
