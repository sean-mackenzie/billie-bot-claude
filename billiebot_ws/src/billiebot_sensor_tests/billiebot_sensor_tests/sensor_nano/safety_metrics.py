"""Pure low-battery-SAFE propagation math for UT-BAT-02 and UT-BAT-02B.

No rclpy, no bag reading -- callers hand in plain timestamp/value sequences pulled from the
rosbag, so the safety scoring is unit-testable without ROS and identical whether it runs
live or against a bag replayed months later.

`SAFE_MODE` mirrors `billiebot_interfaces/msg/MissionStatus.SAFE`. It is duplicated here as
a named constant rather than imported so this module stays import-free; the value is
asserted against the real message constant in the test suite, so the two cannot drift.
"""

import numpy as np

#: billiebot_interfaces/msg/MissionStatus: IDLE=0 PATROL=1 INVESTIGATE=2 TRACK_OBSERVE=3
#: RETURN=4 SAFE=5.
SAFE_MODE = 5

NS_PER_S = 1_000_000_000


def first_crossing_below(times_ns, voltages, threshold_v: float):
    """Timestamp of the first sample strictly below `threshold_v`, or None.

    Strict `<` here describes the *physical measurement* crossing, which is what the latency
    clock starts from. It deliberately says nothing about what comparison the mission
    controller ought to use -- that is UT-BAT-02B's question, and conflating the two would
    hide the BLK-05 boundary defect inside the scorer.
    """
    stamps = np.asarray(times_ns, dtype=np.int64)
    volts = np.asarray(voltages, dtype=np.float64)
    if stamps.size != volts.size:
        raise ValueError(f'times_ns and voltages must be the same length, '
                         f'got {stamps.size} and {volts.size}')
    below = np.flatnonzero(np.isfinite(volts) & (volts < float(threshold_v)))
    if below.size == 0:
        return None
    return int(stamps[below[0]])


def first_mode_entry(times_ns, modes, target_mode: int = SAFE_MODE):
    """Timestamp of the first sample whose mode equals `target_mode`, or None."""
    stamps = np.asarray(times_ns, dtype=np.int64)
    mode_values = np.asarray(modes, dtype=np.int64)
    if stamps.size != mode_values.size:
        raise ValueError(f'times_ns and modes must be the same length, '
                         f'got {stamps.size} and {mode_values.size}')
    hits = np.flatnonzero(mode_values == int(target_mode))
    if hits.size == 0:
        return None
    return int(stamps[hits[0]])


def propagation_latency_s(first_below_ns, first_safe_ns):
    """Seconds from the first below-threshold battery sample to the first SAFE status.

    Returns None when either event is missing. A negative result is returned as-is rather
    than clamped: SAFE that precedes the trigger means the run captured a pre-existing SAFE
    state, which is a test-setup fault the scorer must surface, not smooth over.
    """
    if first_below_ns is None or first_safe_ns is None:
        return None
    return (int(first_safe_ns) - int(first_below_ns)) / NS_PER_S


def mode_before_time(times_ns, modes, cutoff_ns) -> dict:
    """Summarise mission mode strictly before `cutoff_ns`.

    Used for the "at ~10.70 V the mission remains non-SAFE" gate: the high-voltage phase is
    everything before the battery first drops below threshold.
    """
    stamps = np.asarray(times_ns, dtype=np.int64)
    mode_values = np.asarray(modes, dtype=np.int64)
    mask = stamps < int(cutoff_ns)
    selected = mode_values[mask]
    return {
        'sample_count': int(selected.size),
        'safe_count': int(np.sum(selected == SAFE_MODE)),
        'stayed_non_safe': bool(selected.size > 0 and not np.any(selected == SAFE_MODE)),
        'modes_seen': sorted(int(m) for m in np.unique(selected)),
    }


def status_continuity(times_ns, max_gap_s: float) -> dict:
    """Check the mission-status stream never stalls longer than `max_gap_s`.

    A SAFE transition that arrives only because the node froze and restarted is not a
    working safety chain, so continuity is scored alongside the transition itself.
    """
    stamps = np.asarray(times_ns, dtype=np.int64)
    if stamps.size < 2:
        return {'sample_count': int(stamps.size), 'max_gap_s': 0.0,
                'continuous': bool(stamps.size > 0), 'max_gap_limit_s': float(max_gap_s)}
    gaps_s = np.diff(stamps).astype(np.float64) / NS_PER_S
    max_gap = float(np.max(gaps_s))
    return {
        'sample_count': int(stamps.size),
        'max_gap_s': max_gap,
        'continuous': max_gap <= float(max_gap_s),
        'max_gap_limit_s': float(max_gap_s),
    }


def evaluate_threshold_case(case_voltage_v: float, expected_safe: bool, times_ns, modes,
                             window_start_ns: int, window_end_ns: int) -> dict:
    """Score one UT-BAT-02B synthetic-voltage case over its hold window.

    `expected_safe` comes from the **system requirement** SYS-PLT-2 ("SAFE at <= 3.5 V/cell",
    i.e. <= 10.5 V for 3S), never from the current implementation. At exactly 10.5000 V the
    requirement says SAFE and `mission_controller.py`'s strict `<` says otherwise, so this
    case is expected to FAIL until BLK-05 is fixed in production. That failure is the
    intended output of the test: do not special-case it here, and do not soften
    `expected_safe`.
    """
    stamps = np.asarray(times_ns, dtype=np.int64)
    mode_values = np.asarray(modes, dtype=np.int64)
    mask = (stamps >= int(window_start_ns)) & (stamps <= int(window_end_ns))
    observed_modes = mode_values[mask]

    observed_safe = bool(np.any(observed_modes == SAFE_MODE))
    return {
        'case_voltage_v': float(case_voltage_v),
        'expected_safe': bool(expected_safe),
        'observed_safe': observed_safe,
        'sample_count': int(observed_modes.size),
        'modes_seen': sorted(int(m) for m in np.unique(observed_modes)),
        'window_start_ns': int(window_start_ns),
        'window_end_ns': int(window_end_ns),
        'has_samples': bool(observed_modes.size > 0),
        'passed': bool(observed_modes.size > 0 and observed_safe == bool(expected_safe)),
    }


def requirement_expects_safe(case_voltage_v: float, safe_threshold_v: float) -> bool:
    """SYS-PLT-2's rule: SAFE at or below the threshold, i.e. `<=`.

    This is the requirement, written once, in one place. `mission_controller.py:147` uses
    `<`; the difference between this function and that line is precisely BLK-05.
    """
    return float(case_voltage_v) <= float(safe_threshold_v)
