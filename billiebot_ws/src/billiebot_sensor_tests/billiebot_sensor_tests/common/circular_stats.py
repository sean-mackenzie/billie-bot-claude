"""Pure circular statistics for angular data (audio DoA, degrees in [0, 360)).

No hardware/ROS imports.
"""

from typing import Sequence, Union

import numpy as np

ArrayLike = Union[float, Sequence[float], np.ndarray]


def circular_mean_deg(angles_deg: Sequence[float]) -> float:
    angles = np.deg2rad(np.asarray(angles_deg, dtype=np.float64))
    mean_angle = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
    return float(np.rad2deg(mean_angle) % 360.0)


def circular_std_deg(angles_deg: Sequence[float]) -> float:
    angles = np.deg2rad(np.asarray(angles_deg, dtype=np.float64))
    resultant = np.hypot(np.mean(np.cos(angles)), np.mean(np.sin(angles)))
    if resultant <= 1e-12:
        return 180.0
    return float(np.rad2deg(np.sqrt(-2.0 * np.log(resultant))))


def circular_abs_error_deg(measured_deg: ArrayLike, true_deg: ArrayLike) -> ArrayLike:
    """Smallest absolute angular error between measured and true bearings, in [0, 180].

    Handles the 0/360 wraparound correctly, e.g. true=359, measured=1 -> error=2 (not 358).
    """
    measured = np.asarray(measured_deg, dtype=np.float64)
    true = np.asarray(true_deg, dtype=np.float64)
    diff = (measured - true + 180.0) % 360.0 - 180.0
    result = np.abs(diff)
    if np.isscalar(measured_deg) and np.isscalar(true_deg):
        return float(result)
    return result
