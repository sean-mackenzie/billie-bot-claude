"""Pure thermal contrast/bias/noise math. No hardware/ROS imports."""

import numpy as np


def target_background_bias(target_c: np.ndarray, background_c: np.ndarray,
                            ref_target_c: float, ref_background_c: float) -> dict:
    target_c = np.asarray(target_c, dtype=np.float64)
    background_c = np.asarray(background_c, dtype=np.float64)
    return {
        'target_bias_c': float(np.median(target_c) - ref_target_c),
        'background_bias_c': float(np.median(background_c) - ref_background_c),
    }


def contrast_to_noise_ratio(target_c: np.ndarray, background_c: np.ndarray) -> float:
    target_c = np.asarray(target_c, dtype=np.float64)
    background_c = np.asarray(background_c, dtype=np.float64)
    delta_t = float(np.mean(target_c) - np.mean(background_c))
    std_bg = float(np.std(background_c))
    return delta_t / std_bg if std_bg > 0 else float('inf')


def pooled_cnr(target_c: np.ndarray, background_c: np.ndarray) -> float:
    target_c = np.asarray(target_c, dtype=np.float64)
    background_c = np.asarray(background_c, dtype=np.float64)
    delta_t = float(np.mean(target_c) - np.mean(background_c))
    pooled_std = float(np.sqrt(np.std(target_c) ** 2 + np.std(background_c) ** 2))
    return delta_t / pooled_std if pooled_std > 0 else float('inf')


def temporal_noise(frames_over_time: np.ndarray) -> np.ndarray:
    """frames_over_time: (T, N) array of pixel values over time. Returns per-pixel std (N,)."""
    frames = np.asarray(frames_over_time, dtype=np.float64)
    return np.std(frames, axis=0)


def drift(values_over_time: np.ndarray) -> float:
    values = np.asarray(values_over_time, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size < 2:
        return 0.0
    return float(values[-1] - values[0])
