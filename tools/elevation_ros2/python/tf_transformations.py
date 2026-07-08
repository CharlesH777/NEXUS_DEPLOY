"""Minimal tf_transformations compatibility shim for ROS2 Humble bring-up."""

from __future__ import annotations

import numpy as np
from transforms3d.quaternions import quat2mat


def quaternion_matrix(quaternion: list[float] | tuple[float, ...] | np.ndarray) -> np.ndarray:
    """Return a homogeneous rotation matrix from an (x, y, z, w) quaternion."""
    q = np.asarray(quaternion, dtype=np.float64).reshape(4)
    x, y, z, w = q

    if float(np.dot(q, q)) < np.finfo(np.float64).eps:
        return np.identity(4, dtype=np.float64)

    mat = np.identity(4, dtype=np.float64)
    mat[:3, :3] = quat2mat([w, x, y, z])
    return mat
