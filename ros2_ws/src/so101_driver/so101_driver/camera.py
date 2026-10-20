"""Sim camera intrinsics in the pixel convention of CameraInfo and OpenCV (docs/project/05 §3.2).

Pure: no ROS.
"""

import numpy as np


def opencv_intrinsics(K) -> np.ndarray:
    """The provided get_camera_intrinsics() K, principal point moved by -0.5 px [MES-41].

    OpenGL (MuJoCo) puts pixel edges on integers: (W/2, H/2) is the corner of the four central pixels.
    CameraInfo and OpenCV put pixel centers on integers, where that corner is ((W - 1)/2, (H - 1)/2).
    """
    K = np.array(K, dtype=float)
    K[:2, 2] -= 0.5
    return K
