"""Pure battery-divider / ADC math for UT-BAT-01. No rclpy, no hardware imports.

Everything here compares the ROS-reported battery voltage against **independently measured
DMM ground truth**. Nothing in this module ever solves for a calibration constant and feeds
it back into the conversion: the point of UT-BAT-01 is to measure the error of the shipped
`battery_divider_ratio` / `adc_reference_voltage`, and a fit that silently re-derived them
would report a perfect result for a miswired divider. `fit_observed_scale()` exists only to
*report* what the hardware implies, and is explicitly not used to convert anything.
"""

import numpy as np

#: Full-scale count of the ATmega328P's 10-bit ADC. `analogRead()` returns 0..1023, so the
#: conversion divisor is 1023 (the count at V_REF), not 1024.
ADC_MAX_COUNT = 1023.0


def adc_to_volts(adc_counts, adc_reference_voltage: float, divider_ratio: float,
                  adc_max_count: float = ADC_MAX_COUNT) -> np.ndarray:
    """Raw averaged ADC count -> battery volts.

    This is the single conversion used by both the live bridge and the offline analyzer, so
    a replayed bag and a live run can never disagree about what a count meant:

        V_A0  = adc / adc_max_count * adc_reference_voltage
        V_BAT = V_A0 * divider_ratio
    """
    if adc_reference_voltage <= 0:
        raise ValueError(f'adc_reference_voltage must be positive, got {adc_reference_voltage}')
    if divider_ratio <= 0:
        raise ValueError(f'divider_ratio must be positive, got {divider_ratio}')
    counts = np.asarray(adc_counts, dtype=np.float64)
    return counts / float(adc_max_count) * float(adc_reference_voltage) * float(divider_ratio)


def volts_to_adc(battery_volts, adc_reference_voltage: float, divider_ratio: float,
                  adc_max_count: float = ADC_MAX_COUNT) -> np.ndarray:
    """Inverse of adc_to_volts(); used to predict the expected count for a DMM reading."""
    volts = np.asarray(battery_volts, dtype=np.float64)
    return volts / float(divider_ratio) / float(adc_reference_voltage) * float(adc_max_count)


def measured_divider_ratio(v_bat_dmm, v_a0_dmm) -> np.ndarray:
    """Per-point divider ratio implied by the two DMM readings: V_BAT / V_A0.

    This is the physical ratio, measured with a meter and entirely independent of the Nano,
    its ADC and its reference. Comparing it to the configured ratio is what detects a wrong
    resistor pair or a misconfigured `battery_divider_ratio`.
    """
    bat = np.asarray(v_bat_dmm, dtype=np.float64)
    a0 = np.asarray(v_a0_dmm, dtype=np.float64)
    with np.errstate(divide='ignore', invalid='ignore'):
        ratio = np.where(a0 != 0, bat / a0, np.nan)
    return ratio


def linear_fit(x, y) -> dict:
    """Least-squares y = slope*x + intercept, plus fit-quality figures.

    Returns NaNs rather than raising when there are fewer than two points or no spread in x
    -- a single-point sweep is an incomplete test, which the caller reports as such.
    """
    xs = np.asarray(x, dtype=np.float64)
    ys = np.asarray(y, dtype=np.float64)
    if xs.size != ys.size:
        raise ValueError(f'x and y must be the same length, got {xs.size} and {ys.size}')

    finite = np.isfinite(xs) & np.isfinite(ys)
    xs, ys = xs[finite], ys[finite]
    nan_result = {
        'slope': float('nan'), 'intercept': float('nan'), 'rmse': float('nan'),
        'r_squared': float('nan'), 'n_points': int(xs.size),
    }
    if xs.size < 2 or np.ptp(xs) == 0:
        return nan_result

    slope, intercept = np.polyfit(xs, ys, 1)
    predicted = slope * xs + intercept
    residuals = ys - predicted
    rmse = float(np.sqrt(np.mean(residuals ** 2)))
    ss_res = float(np.sum(residuals ** 2))
    ss_tot = float(np.sum((ys - np.mean(ys)) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    return {
        'slope': float(slope),
        'intercept': float(intercept),
        'rmse': rmse,
        'r_squared': r_squared,
        'n_points': int(xs.size),
    }


def error_stats(measured, reference) -> dict:
    """Signed-error statistics of `measured` against DMM `reference`, both in volts."""
    meas = np.asarray(measured, dtype=np.float64)
    ref = np.asarray(reference, dtype=np.float64)
    if meas.size != ref.size:
        raise ValueError(f'measured and reference must be the same length, '
                         f'got {meas.size} and {ref.size}')
    finite = np.isfinite(meas) & np.isfinite(ref)
    errors = meas[finite] - ref[finite]
    if errors.size == 0:
        return {'n_points': 0, 'mean_bias_v': float('nan'), 'rmse_v': float('nan'),
                'max_abs_error_v': float('nan'), 'min_error_v': float('nan'),
                'max_error_v': float('nan')}
    return {
        'n_points': int(errors.size),
        'mean_bias_v': float(np.mean(errors)),
        'rmse_v': float(np.sqrt(np.mean(errors ** 2))),
        'max_abs_error_v': float(np.max(np.abs(errors))),
        'min_error_v': float(np.min(errors)),
        'max_error_v': float(np.max(errors)),
    }


def error_near_threshold(reference, measured, threshold_v: float, window_v: float) -> dict:
    """Error restricted to the safety-threshold neighbourhood |V_ref - threshold| <= window.

    UT-BAT-01 gates this separately from the whole-sweep error because accuracy *at the
    10.5 V SAFE boundary* is what the safety chain actually depends on; a divider that is
    excellent at 12.6 V and biased at 10.5 V is a safety problem the aggregate hides.
    """
    ref = np.asarray(reference, dtype=np.float64)
    meas = np.asarray(measured, dtype=np.float64)
    finite = np.isfinite(ref) & np.isfinite(meas)
    in_window = finite & (np.abs(ref - float(threshold_v)) <= float(window_v))
    if not np.any(in_window):
        return {'n_points': 0, 'max_abs_error_v': float('nan'), 'mean_bias_v': float('nan'),
                'threshold_v': float(threshold_v), 'window_v': float(window_v)}
    errors = meas[in_window] - ref[in_window]
    return {
        'n_points': int(errors.size),
        'max_abs_error_v': float(np.max(np.abs(errors))),
        'mean_bias_v': float(np.mean(errors)),
        'threshold_v': float(threshold_v),
        'window_v': float(window_v),
    }


def adc_monotonicity(input_volts, adc_counts, tolerance_counts: float = 2.0) -> dict:
    """Check that ADC count is non-decreasing as divider input voltage increases.

    Points are sorted by input voltage first, so the operator may sweep the PSU in any order.
    `tolerance_counts` allows genuine quantization/noise wobble -- a 10-bit ADC on a 6:1
    divider moves ~1 count per 29 mV, so a couple of counts of dither between two nearby
    setpoints is expected, while a real inversion (swapped leads, saturation, a wrong pin)
    is far larger.
    """
    volts = np.asarray(input_volts, dtype=np.float64)
    counts = np.asarray(adc_counts, dtype=np.float64)
    if volts.size != counts.size:
        raise ValueError(f'input_volts and adc_counts must be the same length, '
                         f'got {volts.size} and {counts.size}')
    finite = np.isfinite(volts) & np.isfinite(counts)
    volts, counts = volts[finite], counts[finite]
    if volts.size < 2:
        return {'monotonic': True, 'n_points': int(volts.size), 'violations': 0,
                'max_decrease_counts': 0.0}

    order = np.argsort(volts)
    sorted_counts = counts[order]
    deltas = np.diff(sorted_counts)
    decreases = -np.minimum(deltas, 0.0)
    violations = int(np.sum(decreases > float(tolerance_counts)))
    return {
        'monotonic': violations == 0,
        'n_points': int(volts.size),
        'violations': violations,
        'max_decrease_counts': float(np.max(decreases)) if decreases.size else 0.0,
        'tolerance_counts': float(tolerance_counts),
    }


def adc_range_check(adc_counts, adc_max_count: float = ADC_MAX_COUNT) -> dict:
    """Range/saturation check over the whole sweep.

    Saturation at either rail means the divider is scaled wrong for the test range and every
    voltage beyond that point is meaningless, so it is reported separately from accuracy.
    """
    counts = np.asarray(adc_counts, dtype=np.float64)
    if counts.size == 0:
        return {'n_points': 0, 'min_count': float('nan'), 'max_count': float('nan'),
                'saturated_low': False, 'saturated_high': False, 'within_range': False}
    minimum = float(np.min(counts))
    maximum = float(np.max(counts))
    return {
        'n_points': int(counts.size),
        'min_count': minimum,
        'max_count': maximum,
        'saturated_low': minimum <= 0.0,
        'saturated_high': maximum >= float(adc_max_count),
        'within_range': 0.0 <= minimum and maximum <= float(adc_max_count),
    }


def fit_observed_scale(v_bat_dmm, adc_counts, adc_max_count: float = ADC_MAX_COUNT) -> dict:
    """Report the volts-per-count the hardware actually exhibits, and the
    `adc_reference_voltage * divider_ratio` product that would imply.

    REPORTING ONLY. This is never fed back into adc_to_volts(): the test's job is to measure
    the configured conversion's error against physical ground truth, and auto-fitting the
    scale would convert any calibration fault into an apparent pass. Use it to decide,
    deliberately and off-line, whether a configuration value should change -- and then
    re-run the test.
    """
    fit = linear_fit(adc_counts, v_bat_dmm)
    slope = fit['slope']
    implied = slope * float(adc_max_count) if np.isfinite(slope) else float('nan')
    return {
        'volts_per_count': slope,
        'intercept_v': fit['intercept'],
        'implied_reference_times_ratio': implied,
        'rmse_v': fit['rmse'],
        'r_squared': fit['r_squared'],
        'n_points': fit['n_points'],
    }
