"""Synthetic WAV waveform + classifier-event fixtures."""

from dataclasses import dataclass

import numpy as np


def synthetic_wav(kind: str, sample_rate: int = 16000, duration_s: float = 3.0,
                   seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    n = int(sample_rate * duration_s)
    t = np.arange(n) / sample_rate
    if kind == 'silence':
        return np.zeros(n, dtype=np.float32)
    if kind == 'sine':
        return (0.5 * np.sin(2 * np.pi * 440.0 * t)).astype(np.float32)
    if kind == 'clipped':
        wave = 1.5 * np.sin(2 * np.pi * 440.0 * t)
        return np.clip(wave, -1.0, 1.0).astype(np.float32)
    if kind == 'dc_offset':
        return (0.3 + 0.2 * np.sin(2 * np.pi * 440.0 * t)).astype(np.float32)
    if kind == 'white_noise':
        return rng.normal(0.0, 0.1, size=n).astype(np.float32)
    raise ValueError(f'unknown synthetic_wav kind: {kind!r}')


@dataclass
class EventSample:
    t_ns: int
    event_type: int
    confidence: float
    yamnet_label: str
    energy_db: float
    doa_deg: float = 0.0


@dataclass
class StatusSample:
    t_ns: int
    inference_duration_s: float
    top_label: str = ''
    top_score: float = 0.0
    overrun_count: int = 0
