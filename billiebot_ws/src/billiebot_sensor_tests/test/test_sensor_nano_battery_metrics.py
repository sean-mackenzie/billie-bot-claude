"""Battery-divider / ADC metric tests for UT-BAT-01 scoring.

The recurring theme: every accuracy figure is ROS-versus-DMM, and no code path is allowed to
re-derive the divider ratio or ADC reference and feed it back into the conversion. A
miswired divider must score as a failure, not get quietly fitted away.
"""

import numpy as np
import pytest

from fixtures.sensor_nano_fixtures import battery_sweep

from billiebot_sensor_tests.sensor_nano.battery_metrics import (
    ADC_MAX_COUNT,
    adc_monotonicity,
    adc_range_check,
    adc_to_volts,
    error_near_threshold,
    error_stats,
    fit_observed_scale,
    linear_fit,
    measured_divider_ratio,
    volts_to_adc,
)


# --- conversion --------------------------------------------------------------------------

def test_adc_to_volts_uses_the_documented_divider_and_reference():
    # 12.6 V through a 6:1 divider is 2.1 V at A0; against a 5.0 V reference that is
    # 2.1/5.0*1023 = 429.7 counts.
    assert adc_to_volts(429.66, 5.0, 6.0)[()] == pytest.approx(12.6, abs=0.01)


def test_adc_to_volts_and_volts_to_adc_round_trip():
    volts = np.array([9.9, 10.5, 12.6])
    assert adc_to_volts(volts_to_adc(volts, 5.0, 6.0), 5.0, 6.0) == pytest.approx(volts)


def test_zero_counts_convert_to_zero_volts():
    assert adc_to_volts(0.0, 5.0, 6.0)[()] == pytest.approx(0.0)


def test_full_scale_counts_convert_to_reference_times_ratio():
    assert adc_to_volts(ADC_MAX_COUNT, 5.0, 6.0)[()] == pytest.approx(30.0)


@pytest.mark.parametrize('reference,ratio', [(0.0, 6.0), (-5.0, 6.0), (5.0, 0.0), (5.0, -6.0)])
def test_nonsensical_calibration_raises_rather_than_producing_a_voltage(reference, ratio):
    with pytest.raises(ValueError):
        adc_to_volts(500.0, reference, ratio)


# --- divider ratio -----------------------------------------------------------------------

def test_measured_divider_ratio_recovers_a_six_to_one_divider():
    dmm_battery, dmm_a0, _adc, _ros = battery_sweep(divider_ratio=6.0)
    assert measured_divider_ratio(dmm_battery, dmm_a0) == pytest.approx(6.0)


def test_measured_divider_ratio_exposes_a_wrong_resistor_pair():
    # A 5:1 divider configured as 6:1 is exactly the fault UT-BAT-01 exists to catch.
    dmm_battery, dmm_a0, _adc, _ros = battery_sweep(divider_ratio=5.0)
    ratios = measured_divider_ratio(dmm_battery, dmm_a0)
    assert np.mean(ratios) == pytest.approx(5.0)
    assert abs(np.mean(ratios) - 6.0) / 6.0 > 0.02  # fails the 2% required gate


def test_measured_divider_ratio_of_a_zero_midpoint_is_nan_not_infinity():
    ratios = measured_divider_ratio([12.6], [0.0])
    assert np.isnan(ratios[0])


# --- regression --------------------------------------------------------------------------

def test_linear_fit_recovers_a_known_line():
    x = np.array([1.0, 2.0, 3.0, 4.0])
    fit = linear_fit(x, 2.0 * x + 1.0)
    assert fit['slope'] == pytest.approx(2.0)
    assert fit['intercept'] == pytest.approx(1.0)
    assert fit['rmse'] == pytest.approx(0.0, abs=1e-9)
    assert fit['r_squared'] == pytest.approx(1.0)


def test_linear_fit_of_a_single_point_is_nan_not_an_exception():
    fit = linear_fit([1.0], [2.0])
    assert np.isnan(fit['slope']) and fit['n_points'] == 1


def test_linear_fit_ignores_non_finite_pairs():
    fit = linear_fit([1.0, 2.0, np.nan, 4.0], [2.0, 4.0, 5.0, 8.0])
    assert fit['slope'] == pytest.approx(2.0)
    assert fit['n_points'] == 3


def test_linear_fit_length_mismatch_raises():
    with pytest.raises(ValueError):
        linear_fit([1.0, 2.0], [1.0])


def test_r_squared_drops_sharply_for_a_grossly_nonlinear_response():
    x = np.linspace(9.9, 12.6, 8)
    fit = linear_fit(x, x ** 3)
    assert fit['r_squared'] < 0.999


# --- error statistics --------------------------------------------------------------------

def test_error_stats_of_a_perfect_sweep_is_essentially_zero():
    dmm_battery, _a0, _adc, ros = battery_sweep()
    stats = error_stats(ros, dmm_battery)
    # Non-zero only by 10-bit ADC quantization, which is ~0.029 V at a 6:1 divider.
    assert stats['max_abs_error_v'] < 0.05
    assert stats['rmse_v'] < 0.05


def test_error_stats_reports_a_constant_bias():
    dmm_battery, _a0, _adc, ros = battery_sweep(voltage_error=0.25)
    stats = error_stats(ros, dmm_battery)
    assert stats['mean_bias_v'] == pytest.approx(0.25, abs=0.05)
    assert stats['max_abs_error_v'] > 0.20  # fails the provisional accuracy gate


def test_error_stats_of_empty_input_is_nan_not_zero():
    stats = error_stats([], [])
    assert stats['n_points'] == 0 and np.isnan(stats['max_abs_error_v'])


def test_error_stats_length_mismatch_raises():
    with pytest.raises(ValueError):
        error_stats([1.0, 2.0], [1.0])


# --- threshold region --------------------------------------------------------------------

def test_error_near_threshold_only_uses_points_around_the_safe_boundary():
    dmm_battery, _a0, _adc, ros = battery_sweep()
    near = error_near_threshold(dmm_battery, ros, 10.5, 0.30)
    # 10.3, 10.5 and 10.7 are inside +/-0.30 V of 10.5; 9.9, 11.1, 12.0 and 12.6 are not.
    assert near['n_points'] == 3


def test_error_near_threshold_isolates_a_bias_local_to_the_safety_region():
    # A divider that is fine at 12.6 V but biased at 10.5 V is a safety problem the
    # whole-sweep aggregate would hide, which is why this gate exists separately.
    dmm_battery = np.array([9.9, 10.3, 10.5, 10.7, 12.6])
    ros = dmm_battery.copy()
    ros[1:4] += 0.4
    near = error_near_threshold(dmm_battery, ros, 10.5, 0.30)
    assert near['max_abs_error_v'] == pytest.approx(0.4)
    assert error_stats(ros, dmm_battery)['mean_bias_v'] < 0.4  # aggregate dilutes it


def test_error_near_threshold_with_no_points_in_window_reports_zero_points():
    # The analyzer treats this as a failure: the safety-critical region was never measured.
    near = error_near_threshold([12.0, 12.6], [12.0, 12.6], 10.5, 0.10)
    assert near['n_points'] == 0 and np.isnan(near['max_abs_error_v'])


# --- monotonicity and range --------------------------------------------------------------

def test_adc_monotonicity_accepts_a_clean_rising_sweep():
    dmm_battery, _a0, adc, _ros = battery_sweep()
    assert adc_monotonicity(dmm_battery, adc)['monotonic']


def test_adc_monotonicity_is_order_independent():
    # The operator may sweep the PSU down, or revisit a point; sorting by input voltage
    # first is what makes that legitimate.
    dmm_battery, _a0, adc, _ros = battery_sweep()
    order = np.array([3, 0, 6, 2, 5, 1, 4])
    assert adc_monotonicity(dmm_battery[order], adc[order])['monotonic']


def test_adc_monotonicity_tolerates_quantization_dither():
    volts = np.array([10.3, 10.5, 10.7])
    counts = np.array([351.0, 350.0, 365.0])  # one count backwards
    assert adc_monotonicity(volts, counts, tolerance_counts=2.0)['monotonic']


def test_adc_monotonicity_flags_a_real_inversion():
    volts = np.array([9.9, 10.5, 11.1, 12.6])
    counts = np.array([337.0, 358.0, 300.0, 429.0])  # 58 counts backwards
    result = adc_monotonicity(volts, counts, tolerance_counts=2.0)
    assert not result['monotonic']
    assert result['violations'] == 1
    assert result['max_decrease_counts'] == pytest.approx(58.0)


def test_adc_monotonicity_length_mismatch_raises():
    with pytest.raises(ValueError):
        adc_monotonicity([1.0, 2.0], [1.0])


def test_adc_range_check_accepts_an_unsaturated_sweep():
    _dmm, _a0, adc, _ros = battery_sweep()
    result = adc_range_check(adc)
    assert result['within_range'] and not result['saturated_high']


def test_adc_range_check_flags_saturation_at_the_top_rail():
    result = adc_range_check(np.array([500.0, 1023.0]))
    assert result['saturated_high'] and not result['saturated_low']


def test_adc_range_check_flags_a_grounded_input():
    result = adc_range_check(np.array([0.0, 100.0]))
    assert result['saturated_low']


# --- observed scale is reporting only ----------------------------------------------------

def test_fit_observed_scale_reports_the_hardware_implied_product():
    dmm_battery, _a0, adc, _ros = battery_sweep(divider_ratio=6.0, adc_reference=5.0)
    observed = fit_observed_scale(dmm_battery, adc)
    assert observed['implied_reference_times_ratio'] == pytest.approx(30.0, rel=0.01)
    assert observed['r_squared'] == pytest.approx(1.0, abs=1e-3)


def test_fit_observed_scale_does_not_change_what_adc_to_volts_returns():
    # The guard against auto-calibration: reporting the observed scale must have no effect
    # on the conversion the verdict is computed from.
    dmm_battery, _a0, adc, ros_before = battery_sweep(divider_ratio=5.0)
    fit_observed_scale(dmm_battery, adc)
    ros_after = adc_to_volts(adc, 5.0, 5.0)
    assert ros_after == pytest.approx(ros_before)


def test_a_wrong_configured_ratio_still_produces_a_failing_error():
    # Hardware is 5:1 but the bridge is configured 6:1. The reported voltage must be wrong
    # by ~20%, not silently corrected.
    dmm_battery, _a0, adc, _ros = battery_sweep(divider_ratio=5.0)
    reported = adc_to_volts(adc, 5.0, 6.0)
    stats = error_stats(reported, dmm_battery)
    assert stats['max_abs_error_v'] > 1.0
