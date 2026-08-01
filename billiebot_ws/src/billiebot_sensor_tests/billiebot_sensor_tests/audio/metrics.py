"""Pure audio-signal-quality math. No hardware imports."""

import numpy as np


def rms_dbfs(samples: np.ndarray) -> float:
    samples = np.asarray(samples, dtype=np.float64)
    rms = float(np.sqrt(np.mean(samples ** 2))) if samples.size else 0.0
    return 20.0 * np.log10(max(rms, 1e-12))


def peak_dbfs(samples: np.ndarray) -> float:
    samples = np.asarray(samples, dtype=np.float64)
    peak = float(np.max(np.abs(samples))) if samples.size else 0.0
    return 20.0 * np.log10(max(peak, 1e-12))


def clipping_fraction(samples: np.ndarray, threshold: float = 0.99) -> float:
    samples = np.asarray(samples, dtype=np.float64)
    if samples.size == 0:
        return 0.0
    clipped = np.abs(samples) >= threshold
    return float(np.sum(clipped)) / samples.size


def dc_offset(samples: np.ndarray) -> float:
    samples = np.asarray(samples, dtype=np.float64)
    return float(np.mean(samples)) if samples.size else 0.0


def channel_correlation(channel_a: np.ndarray, channel_b: np.ndarray) -> float:
    a = np.asarray(channel_a, dtype=np.float64)
    b = np.asarray(channel_b, dtype=np.float64)
    if np.std(a) == 0 or np.std(b) == 0:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def mains_hum_energy(samples: np.ndarray, sample_rate: int, freqs=(50.0, 60.0),
                      bandwidth_hz: float = 2.0) -> dict:
    """Fraction of total spectral energy near each mains frequency, via numpy FFT
    (no scipy dependency)."""
    samples = np.asarray(samples, dtype=np.float64)
    n = samples.size
    if n == 0:
        return {f: 0.0 for f in freqs}
    spectrum = np.abs(np.fft.rfft(samples))
    freq_bins = np.fft.rfftfreq(n, d=1.0 / sample_rate)
    total_energy = float(np.sum(spectrum ** 2)) or 1.0
    result = {}
    for f in freqs:
        mask = np.abs(freq_bins - f) <= bandwidth_hz
        band_energy = float(np.sum(spectrum[mask] ** 2))
        result[f] = band_energy / total_energy
    return result
