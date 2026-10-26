"""sim_scene - our sim scenes: a copy of the provided scene edited with mujoco.MjSpec, never sim/ (ADR-036).

generate() copies the provided scene folder (its include and mesh paths resolve from the main file), edits the
copy and writes the result next to it; the driver loads it through its scene_path parameter. SimScene is the
launch substitution that does it at launch time, in sim only.

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

MARKER_PIXELS = 600  # texture resolution of a marker's black square


def marker_corners(center, side):
    """A marker's corners in base_link (on the table), in the order cv2.aruco detects them: seen from above,
    the marker image's up is +x and its right is -y."""
    x, y, h = center[0], center[1], side / 2
    return np.array([[x + h, y + h, 0.0], [x + h, y - h, 0.0], [x - h, y - h, 0.0], [x - h, y + h, 0.0]])


def generate(folder, markers=None) -> Path:
    """Our scene in folder (a copy of the provided one, edited); returns the path of its main file."""
    folder = Path(folder) / "so101"
    shutil.copytree(Path(DEFAULT_SCENE).parent, folder)
    spec = mujoco.MjSpec.from_file(str(folder / Path(DEFAULT_SCENE).name))
    if markers:
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, markers["dictionary"]))
        border = round(MARKER_PIXELS * (markers["plane_m"] - markers["side_m"]) / markers["side_m"] / 2)
        half = markers["plane_m"] / 2
        for i, (x, y) in markers["markers"].items():
            image = cv2.aruco.generateImageMarker(dictionary, i, MARKER_PIXELS)
            image = cv2.copyMakeBorder(image, *(border,) * 4, cv2.BORDER_CONSTANT, value=255)  # quiet zone
            cv2.imwrite(str(folder / f"aruco_{i}.png"), np.rot90(image, -1))  # see marker_corners
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
    """The path of our scene, generated at launch: empty (the provided scene) on the real arm, or in sim
    without calibration."""

    def __init__(self, use_sim, calibration):
        super().__init__()
        self._use_sim, self._calibration = use_sim, calibration

    def describe(self):
        return "SimScene(use_sim, calibration)"

    def perform(self, context):
        if perform_substitutions(context, [self._use_sim]).lower() != "true":
            return ""
        if perform_substitutions(context, [self._calibration]).lower() != "true":
            return ""
        config = Path(get_package_share_directory("so101_bringup")) / "config"
        markers = yaml.safe_load((config / "markers_sim.yaml").read_text())
        return str(generate(tempfile.mkdtemp(prefix="so101_scene_"), markers=markers))
