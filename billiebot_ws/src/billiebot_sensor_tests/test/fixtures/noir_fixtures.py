"""Synthetic NoIR chart image fixtures. No cv2/scipy dependency — hand-rolled blur kernel."""

import numpy as np


def _box_blur(img: np.ndarray, passes: int = 3) -> np.ndarray:
    kernel = np.ones((3, 3), dtype=np.float64) / 9.0
    out = img.astype(np.float64)
    for _ in range(passes):
        padded = np.pad(out, 1, mode='edge')
        blurred = np.zeros_like(out)
        for i in range(3):
            for j in range(3):
                blurred += kernel[i, j] * padded[i:i + out.shape[0], j:j + out.shape[1]]
        out = blurred
    return out


def synthetic_chart(width: int = 64, height: int = 64, sharp: bool = True,
                     contrast: float = 1.0, clip: bool = False, noise_std: float = 0.0,
                     seed: int = 0) -> np.ndarray:
    """Half-black / half-white chart (a minimal contrast-to-noise + sharpness target),
    optionally blurred, contrast-scaled, clipped, and noised. Returns (H, W, 3) uint8."""
    rng = np.random.default_rng(seed)
    base = np.zeros((height, width), dtype=np.float64)
    base[:, width // 2:] = 255.0
    base = 127.5 + (base - 127.5) * contrast
    if not sharp:
        base = _box_blur(base, passes=3)
    if noise_std > 0:
        base = base + rng.normal(0.0, noise_std, size=base.shape)
    base = np.clip(base, 0, 255)
    if clip:
        base[:, :4] = 0
        base[:, -4:] = 255
    gray = base.astype(np.uint8)
    return np.repeat(gray[:, :, None], 3, axis=2)
