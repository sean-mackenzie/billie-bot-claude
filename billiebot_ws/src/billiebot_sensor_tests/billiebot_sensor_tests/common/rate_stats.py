"""Pure rate/gap/timestamp-integrity statistics. No rclpy, no hardware imports."""

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class RateStats:
    count: int
    first_ts_ns: int
    last_ts_ns: int
    mean_hz: float
    median_hz: float
    min_period_s: float
    max_period_s: float
    period_std_s: float
    max_gap_s: float
    monotonic: bool

    def to_dict(self) -> dict:
        return {
            'count': self.count,
            'first_ts_ns': self.first_ts_ns,
            'last_ts_ns': self.last_ts_ns,
            'mean_hz': self.mean_hz,
            'median_hz': self.median_hz,
            'min_period_s': self.min_period_s,
            'max_period_s': self.max_period_s,
            'period_std_s': self.period_std_s,
            'max_gap_s': self.max_gap_s,
            'monotonic': self.monotonic,
        }


def compute_rate_stats(timestamps_ns: Sequence[int]) -> RateStats:
    """Compute rate/gap/monotonicity statistics from a sequence of nanosecond timestamps.

    Timestamps are consumed in the order given (receipt order), not sorted, so that a
    nonmonotonic sequence is actually detected rather than silently corrected.
    """
    n = len(timestamps_ns)
    if n == 0:
        return RateStats(0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True)
    if n == 1:
        t0 = int(timestamps_ns[0])
        return RateStats(1, t0, t0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True)

    ts = np.asarray(timestamps_ns, dtype=np.int64)
    diffs_ns = np.diff(ts)
    monotonic = bool(np.all(diffs_ns > 0))
    periods_s = diffs_ns.astype(np.float64) / 1.0e9

    mean_period = float(np.mean(periods_s))
    median_period = float(np.median(periods_s))

    return RateStats(
        count=n,
        first_ts_ns=int(ts[0]),
        last_ts_ns=int(ts[-1]),
        mean_hz=(1.0 / mean_period) if mean_period > 0 else 0.0,
        median_hz=(1.0 / median_period) if median_period > 0 else 0.0,
        min_period_s=float(np.min(periods_s)),
        max_period_s=float(np.max(periods_s)),
        period_std_s=float(np.std(periods_s)),
        max_gap_s=float(np.max(periods_s)),
        monotonic=monotonic,
    )
