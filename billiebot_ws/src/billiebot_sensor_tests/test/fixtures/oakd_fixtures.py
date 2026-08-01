"""Synthetic OAK-D depth-plane and detection-segment fixtures."""

import numpy as np

# Canonical dataclasses live in the installed analysis module, not here — test fixtures
# depend on production/analysis code, never the other way around.
from billiebot_sensor_tests.oakd.detection_scoring import (  # noqa: F401
    DetectionSample,
    GroundTruthSegment,
)


def synthetic_plane(distance_m: float, width: int = 60, height: int = 60,
                     tilt_deg: float = 0.0, noise_std_m: float = 0.0,
                     hole_fraction: float = 0.0, outlier_fraction: float = 0.0,
                     bias_m: float = 0.0, seed: int = 0) -> np.ndarray:
    """Returns an (N,3) array of XYZ points (metres) approximating a flat target ROI,
    optionally tilted about the X axis, with Gaussian noise, invalid holes (NaN pixels),
    and gross outliers injected."""
    rng = np.random.default_rng(seed)
    xs = np.linspace(-0.3, 0.3, width)
    ys = np.linspace(-0.3, 0.3, height)
    xx, yy = np.meshgrid(xs, ys)
    tilt = np.deg2rad(tilt_deg)
    zz = distance_m + bias_m + yy * np.tan(tilt)
    if noise_std_m > 0:
        zz = zz + rng.normal(0.0, noise_std_m, size=zz.shape)
    points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)

    n = points.shape[0]
    if hole_fraction > 0:
        n_holes = int(n * hole_fraction)
        idx = rng.choice(n, size=n_holes, replace=False)
        points[idx, 2] = np.nan
    if outlier_fraction > 0:
        n_out = int(n * outlier_fraction)
        idx = rng.choice(n, size=n_out, replace=False)
        points[idx, 2] += rng.choice([-1.0, 1.0], size=n_out) * rng.uniform(1.0, 3.0, size=n_out)
    return points
