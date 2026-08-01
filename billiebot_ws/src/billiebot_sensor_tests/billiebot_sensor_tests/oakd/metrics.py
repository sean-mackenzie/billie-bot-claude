"""Pure OAK-D depth-accuracy math. No hardware/ROS imports."""

import numpy as np


def valid_pixel_fraction(depth_m: np.ndarray, min_m: float = 0.1, max_m: float = 10.0) -> float:
    depth_m = np.asarray(depth_m, dtype=np.float64)
    if depth_m.size == 0:
        return 0.0
    valid = np.isfinite(depth_m) & (depth_m >= min_m) & (depth_m <= max_m)
    return float(np.sum(valid)) / depth_m.size


def robust_std_mad(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return 0.0
    median = np.median(values)
    mad = np.median(np.abs(values - median))
    return float(1.4826 * mad)


def percentiles(values: np.ndarray, ps=(5, 50, 95)) -> dict:
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {p: float('nan') for p in ps}
    return {p: float(np.percentile(values, p)) for p in ps}


def fraction_within_band(values: np.ndarray, ground_truth_m: float, band_m: float = 0.20) -> float:
    values = np.asarray(values, dtype=np.float64)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 0.0
    within = np.abs(finite - ground_truth_m) <= band_m
    return float(np.sum(within)) / finite.size


def fit_plane_svd(points_xyz: np.ndarray):
    """Fits a plane to Nx3 points via SVD. Returns (normal (3,), centroid (3,))."""
    points = np.asarray(points_xyz, dtype=np.float64)
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < 3:
        raise ValueError('need at least 3 finite points to fit a plane')
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    return normal, centroid


def point_to_plane_rmse(points_xyz: np.ndarray, normal: np.ndarray, centroid: np.ndarray) -> float:
    points = np.asarray(points_xyz, dtype=np.float64)
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] == 0:
        return 0.0
    residuals = (points - centroid) @ normal
    return float(np.sqrt(np.mean(residuals ** 2)))


def plane_normal_angle_deg(normal: np.ndarray, reference=(0.0, 0.0, 1.0)) -> float:
    normal = np.asarray(normal, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    reference = reference / np.linalg.norm(reference)
    cos_angle = np.clip(np.abs(np.dot(normal, reference)), -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cos_angle)))
