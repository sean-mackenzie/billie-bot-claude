"""Synthetic frames for the visualization-preview tests. No hardware, no ROS, no cv2."""

import numpy as np


def synthetic_bgr(width: int = 1920, height: int = 1080, seed: int = 0) -> np.ndarray:
    """Textured BGR frame with smooth gradients, noise, and a hard-edged patch -- roughly the
    entropy of a real indoor scene, so JPEG size assertions are not measuring a flat fill."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    base = np.sin(xx / 23.0) * 40 + np.cos(yy / 17.0) * 40 + 128
    bgr = np.stack([base * 0.6 + 40, base * 0.8 + 20, base], axis=-1)
    bgr += rng.normal(0, 12, bgr.shape)
    bgr[height // 3:height // 2, width // 4:width // 2] = 200
    return np.clip(bgr, 0, 255).astype(np.uint8)


def synthetic_red_bgr(width: int = 64, height: int = 64) -> np.ndarray:
    """Saturated red frame in BGR channel order (B=0, G=0, R=255) -- used to prove the
    BGR->RGB swap survives compression rather than silently producing a blue image."""
    bgr = np.zeros((height, width, 3), dtype=np.uint8)
    bgr[:, :, 2] = 255
    return bgr


def synthetic_depth_mm(width: int = 640, height: int = 400,
                       near_mm: int = 500, far_mm: int = 4500,
                       hole_slice=None) -> np.ndarray:
    """Depth ramp in millimetres, left-to-right from `near_mm` to `far_mm`, matching the OAK-D's
    16UC1 units. `hole_slice` paints an invalid (0 mm) region, DepthAI's no-measurement value."""
    ramp = np.linspace(near_mm, far_mm, width, dtype=np.float32)
    depth = np.tile(ramp, (height, 1)).astype(np.uint16)
    if hole_slice is not None:
        depth[hole_slice] = 0
    return depth


def synthetic_depth_plane_mm(width: int = 640, height: int = 400,
                             distance_mm: int = 2000) -> np.ndarray:
    """Flat plane at a fixed distance -- the UT-OAK-02 mock scene."""
    return np.full((height, width), distance_mm, dtype=np.uint16)
