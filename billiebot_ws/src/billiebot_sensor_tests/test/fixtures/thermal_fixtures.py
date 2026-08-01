"""Synthetic MLX90640 32x24 thermal-frame fixtures."""

import numpy as np


def synthetic_thermal_frame(ambient_c: float = 22.0, blobs=(), noise_std_c: float = 0.0,
                             nan_pixels=(), width: int = 32, height: int = 24,
                             seed: int = 0) -> np.ndarray:
    """blobs: iterable of (cx, cy, radius_px, temp_c). Returns a flat (width*height,) array
    matching thermal_node.py's row-major frame[y*32+x] layout, so it can stand in directly
    for a raw MLX90640 frame."""
    rng = np.random.default_rng(seed)
    frame = np.full((height, width), ambient_c, dtype=np.float64)
    for cx, cy, radius, temp_c in blobs:
        yy, xx = np.mgrid[0:height, 0:width]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2
        frame[mask] = temp_c
    if noise_std_c > 0:
        frame = frame + rng.normal(0.0, noise_std_c, size=frame.shape)
    flat = frame.flatten()
    for idx in nan_pixels:
        flat[idx] = np.nan
    return flat
