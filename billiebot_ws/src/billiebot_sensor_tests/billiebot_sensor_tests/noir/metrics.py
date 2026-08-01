"""Pure NoIR image-quality math. No hardware/ROS/cv2/scipy imports — hand-rolled kernels."""

import numpy as np

_LAPLACIAN_KERNEL = np.array([[0, 1, 0], [1, -4, 1], [0, 1, 0]], dtype=np.float64)


def contrast_to_noise_ratio(white_patch: np.ndarray, black_patch: np.ndarray) -> float:
    white = np.asarray(white_patch, dtype=np.float64)
    black = np.asarray(black_patch, dtype=np.float64)
    denom = float(np.sqrt(np.std(white) ** 2 + np.std(black) ** 2))
    return float(np.abs(np.mean(white) - np.mean(black)) / denom) if denom > 0 else float('inf')


def laplacian_variance(gray_image: np.ndarray) -> float:
    """Hand-rolled 3x3 Laplacian convolution (no scipy/cv2) as a sharpness proxy — higher
    variance of the Laplacian response means a sharper image."""
    img = np.asarray(gray_image, dtype=np.float64)
    padded = np.pad(img, 1, mode='edge')
    response = np.zeros_like(img)
    for i in range(3):
        for j in range(3):
            weight = _LAPLACIAN_KERNEL[i, j]
            if weight == 0:
                continue
            response += weight * padded[i:i + img.shape[0], j:j + img.shape[1]]
    return float(np.var(response))


def clipping_fraction(image: np.ndarray, low: int = 0, high: int = 255) -> float:
    arr = np.asarray(image)
    if arr.size == 0:
        return 0.0
    clipped = (arr <= low) | (arr >= high)
    return float(np.sum(clipped)) / arr.size


def temporal_stability(values_over_time) -> float:
    """Coefficient of variation (std/mean) of a metric sampled over a stable interval —
    used for both the brightness and sharpness CoV acceptance checks."""
    values = np.asarray(values_over_time, dtype=np.float64)
    mean = np.mean(values)
    if mean == 0:
        return float('inf')
    return float(np.std(values) / abs(mean))
