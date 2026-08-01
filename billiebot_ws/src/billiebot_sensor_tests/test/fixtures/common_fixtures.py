"""Synthetic timestamp sequences for common/rate_stats.py tests."""

import numpy as np


def ideal_timestamps(n: int, hz: float, start_ns: int = 0) -> list:
    period_ns = int(round(1e9 / hz))
    return [start_ns + i * period_ns for i in range(n)]


def jittered_timestamps(n: int, hz: float, jitter_std_s: float, seed: int = 0,
                         start_ns: int = 0) -> list:
    rng = np.random.default_rng(seed)
    period_ns = 1e9 / hz
    base = start_ns + np.arange(n) * period_ns
    jitter_ns = rng.normal(0.0, jitter_std_s * 1e9, size=n)
    ts = np.sort(base + jitter_ns)  # jitter alone should not create nonmonotonicity here
    return [int(t) for t in ts]


def nonmonotonic_timestamps(n: int, hz: float, swap_index: int = 2,
                             start_ns: int = 0) -> list:
    ts = ideal_timestamps(n, hz, start_ns)
    if 0 < swap_index < len(ts):
        ts[swap_index - 1], ts[swap_index] = ts[swap_index], ts[swap_index - 1]
    return ts


def gapped_timestamps(n: int, hz: float, gap_index: int, gap_s: float,
                       start_ns: int = 0) -> list:
    ts = ideal_timestamps(n, hz, start_ns)
    gap_ns = int(gap_s * 1e9)
    for i in range(gap_index, len(ts)):
        ts[i] += gap_ns
    return ts
