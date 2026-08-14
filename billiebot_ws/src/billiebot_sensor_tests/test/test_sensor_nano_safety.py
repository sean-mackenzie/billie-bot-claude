"""Low-battery SAFE propagation tests for UT-BAT-02 and UT-BAT-02B scoring.

The important assertions here are the ones about the 10.5000 V boundary. SYS-PLT-2 requires
SAFE at `<= 3.5 V/cell`; `mission_controller.py:147` implements `<`. These tests encode the
REQUIREMENT, so that when the production comparison is eventually fixed they keep passing,
and until then UT-BAT-02B correctly reports a failure against the real controller.
"""

import numpy as np
import pytest

from fixtures.sensor_nano_fixtures import battery_voltage_series, mission_status_series

from billiebot_sensor_tests.sensor_nano.safety_metrics import (
    NS_PER_S,
    SAFE_MODE,
    evaluate_threshold_case,
    first_crossing_below,
    first_mode_entry,
    mode_before_time,
    propagation_latency_s,
    requirement_expects_safe,
    status_continuity,
)


def test_safe_mode_constant_matches_the_interface_definition():
    # billiebot_interfaces/msg/MissionStatus declares SAFE=5. safety_metrics duplicates the
    # value to stay import-free, so this is the guard against the two drifting apart.
    assert SAFE_MODE == 5


# --- SYS-PLT-2 boundary ------------------------------------------------------------------

@pytest.mark.parametrize('voltage,expected', [
    (10.5001, False),
    (10.5000, True),   # the requirement says SAFE; production's strict `<` disagrees (BLK-05)
    (10.4999, True),
    (12.6, False),
    (9.9, True),
])
def test_requirement_expects_safe_implements_less_than_or_equal(voltage, expected):
    assert requirement_expects_safe(voltage, 10.5) is expected


def test_the_requirement_and_a_strict_comparison_differ_only_at_the_boundary():
    # This test *is* BLK-05, written out: the two rules agree everywhere except 10.5000 V.
    for voltage in (10.4999, 10.5000, 10.5001):
        requirement = requirement_expects_safe(voltage, 10.5)
        production_style = voltage < 10.5
        if voltage == 10.5000:
            assert requirement is True and production_style is False
        else:
            assert requirement == production_style


def test_boundary_voltages_survive_float32_conversion_distinctly():
    # BatteryState.voltage is float32; if the three cases collapsed onto one another the
    # test would be meaningless. 10.5 is exactly representable, the neighbours are not equal
    # to it.
    values = np.array([10.5001, 10.5, 10.4999], dtype=np.float32)
    assert values[0] > np.float32(10.5)
    assert values[1] == np.float32(10.5)
    assert values[2] < np.float32(10.5)


# --- crossing detection ------------------------------------------------------------------

def test_first_crossing_below_finds_the_step():
    stamps, voltages = battery_voltage_series(20, step_index=10)
    assert first_crossing_below(stamps, voltages, 10.5) == stamps[10]


def test_first_crossing_below_returns_none_when_the_battery_never_drops():
    stamps, voltages = battery_voltage_series(20, step_index=None)
    assert first_crossing_below(stamps, voltages, 10.5) is None


def test_first_crossing_below_ignores_non_finite_samples():
    stamps = np.array([0, 1, 2], dtype=np.int64)
    voltages = np.array([np.nan, 10.7, 10.3])
    assert first_crossing_below(stamps, voltages, 10.5) == 2


def test_first_crossing_below_length_mismatch_raises():
    with pytest.raises(ValueError):
        first_crossing_below([0, 1], [10.0], 10.5)


def test_first_mode_entry_finds_the_safe_transition():
    stamps, modes = mission_status_series(20, safe_from_index=12)
    assert first_mode_entry(stamps, modes, SAFE_MODE) == stamps[12]


def test_first_mode_entry_returns_none_when_safe_never_occurs():
    stamps, modes = mission_status_series(20, safe_from_index=None)
    assert first_mode_entry(stamps, modes, SAFE_MODE) is None


# --- latency -----------------------------------------------------------------------------

def test_propagation_latency_is_the_gap_between_the_two_events():
    assert propagation_latency_s(1 * NS_PER_S, 2 * NS_PER_S) == pytest.approx(1.0)


def test_propagation_latency_is_none_when_either_event_is_missing():
    assert propagation_latency_s(None, 2 * NS_PER_S) is None
    assert propagation_latency_s(1 * NS_PER_S, None) is None


def test_a_negative_latency_is_reported_rather_than_clamped():
    # SAFE before the trigger means the run started already SAFE -- a setup fault the scorer
    # must surface, which is why the analyzer requires 0 <= latency <= limit.
    assert propagation_latency_s(2 * NS_PER_S, 1 * NS_PER_S) == pytest.approx(-1.0)


def test_end_to_end_latency_of_a_realistic_run_is_under_the_two_second_gate():
    battery_stamps, voltages = battery_voltage_series(60, hz=5.0, step_index=30)
    # 2 Hz mission ticks, SAFE on the tick after the battery step.
    status_stamps, modes = mission_status_series(30, hz=2.0, safe_from_index=13)
    below = first_crossing_below(battery_stamps, voltages, 10.5)
    safe = first_mode_entry(status_stamps, modes, SAFE_MODE)
    assert 0.0 <= propagation_latency_s(below, safe) <= 2.0


# --- high-voltage phase ------------------------------------------------------------------

def test_mode_before_time_confirms_the_mission_stayed_non_safe_at_high_voltage():
    stamps, modes = mission_status_series(20, mode=1, safe_from_index=12)
    phase = mode_before_time(stamps, modes, stamps[12])
    assert phase['stayed_non_safe'] and phase['safe_count'] == 0
    assert phase['modes_seen'] == [1]


def test_mode_before_time_catches_a_mission_already_safe_at_high_voltage():
    stamps, modes = mission_status_series(20, mode=1, safe_from_index=0)
    assert not mode_before_time(stamps, modes, stamps[10])['stayed_non_safe']


def test_mode_before_time_with_no_prior_samples_does_not_claim_success():
    stamps, modes = mission_status_series(5, mode=1)
    phase = mode_before_time(stamps, modes, stamps[0])
    assert phase['sample_count'] == 0 and not phase['stayed_non_safe']


# --- continuity --------------------------------------------------------------------------

def test_status_continuity_accepts_a_steady_two_hertz_stream():
    stamps, _modes = mission_status_series(40, hz=2.0)
    assert status_continuity(stamps, 2.0)['continuous']


def test_status_continuity_flags_a_stall():
    stamps, _modes = mission_status_series(10, hz=2.0)
    stalled = np.concatenate([stamps, stamps[-1:] + 5 * NS_PER_S])
    result = status_continuity(stalled, 2.0)
    assert not result['continuous']
    assert result['max_gap_s'] == pytest.approx(5.0)


def test_status_continuity_of_a_single_sample_is_not_called_continuous_by_accident():
    result = status_continuity(np.array([0], dtype=np.int64), 2.0)
    assert result['sample_count'] == 1 and result['max_gap_s'] == 0.0


# --- UT-BAT-02B case evaluation ----------------------------------------------------------

def test_case_passes_when_safe_is_observed_and_required():
    stamps, modes = mission_status_series(20, safe_from_index=5)
    result = evaluate_threshold_case(10.4999, True, stamps, modes, stamps[0], stamps[-1])
    assert result['observed_safe'] and result['passed']


def test_case_passes_when_safe_is_correctly_absent():
    stamps, modes = mission_status_series(20, mode=1, safe_from_index=None)
    result = evaluate_threshold_case(10.5001, False, stamps, modes, stamps[0], stamps[-1])
    assert not result['observed_safe'] and result['passed']


def test_the_exact_boundary_case_fails_against_a_strict_less_than_controller():
    # This models today's production behaviour: at 10.5000 V a `<` comparison never trips,
    # so the mission stays in PATROL while SYS-PLT-2 requires SAFE. UT-BAT-02B must report
    # this as a FAILURE -- that is the point of the test, and the expectation must not be
    # softened to make the suite green.
    stamps, modes = mission_status_series(20, mode=1, safe_from_index=None)
    result = evaluate_threshold_case(10.5000, True, stamps, modes, stamps[0], stamps[-1])
    assert result['expected_safe'] is True
    assert result['observed_safe'] is False
    assert result['passed'] is False


def test_the_exact_boundary_case_would_pass_once_the_production_comparison_is_fixed():
    # Forward-looking guard: when mission_controller adopts `<=`, SAFE will be observed at
    # 10.5000 V and this scorer must accept it with no change here.
    stamps, modes = mission_status_series(20, safe_from_index=4)
    result = evaluate_threshold_case(10.5000, True, stamps, modes, stamps[0], stamps[-1])
    assert result['passed']


def test_a_case_window_with_no_samples_does_not_pass_by_default():
    stamps, modes = mission_status_series(20, mode=1)
    far_future = int(stamps[-1]) + 100 * NS_PER_S
    result = evaluate_threshold_case(10.5001, False, stamps, modes, far_future,
                                      far_future + NS_PER_S)
    assert result['sample_count'] == 0
    assert not result['has_samples']
    assert not result['passed']


def test_case_evaluation_only_looks_inside_its_own_window():
    # SAFE from a previous case must not leak into the next one's verdict.
    stamps, modes = mission_status_series(40, mode=1, safe_from_index=0)
    modes[20:] = 1  # reset via /set_mode partway through
    result = evaluate_threshold_case(10.5001, False, stamps, modes, stamps[20], stamps[-1])
    assert not result['observed_safe'] and result['passed']


def test_case_records_every_mode_seen_for_the_report():
    stamps, modes = mission_status_series(20, mode=1, safe_from_index=10)
    result = evaluate_threshold_case(10.4999, True, stamps, modes, stamps[0], stamps[-1])
    assert result['modes_seen'] == [1, 5]
