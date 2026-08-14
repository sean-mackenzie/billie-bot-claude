#!/usr/bin/env python3
"""CLI: `score_battery_safe` -- low-battery SAFE propagation scoring.

    --profile physical   UT-BAT-02:  real Sensor Nano -> /battery_state -> production
                                     mission_controller -> /billiebot/mission_status -> SAFE
    --profile threshold  UT-BAT-02B: synthetic BatteryState across the exact 10.5 V boundary

The two profiles are scored separately and reported under separate test IDs precisely so a
hardware result and a software-requirement result can never be confused for one another
(test plan section 18.1).

The `--high-voltage-v` / `--low-voltage-v` / `--safe-threshold-v` arguments are optional
overrides; each falls back to `sensor_bench.yaml`. That is what lets the standard
`run_sensor_test` orchestrator -- which only ever forwards --results-dir, --config-file and
--profile -- drive this scorer, while the manual command in the test plan still works.
"""

import argparse
import csv
import os
import sys

import numpy as np

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.report import (
    render_report_md, write_metrics_csv, write_metrics_json,
)
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.sensor_nano.battery_threshold_test import CSV_NAME as CASES_CSV
from billiebot_sensor_tests.sensor_nano.safety_metrics import (
    SAFE_MODE,
    evaluate_threshold_case,
    first_crossing_below,
    first_mode_entry,
    mode_before_time,
    propagation_latency_s,
    status_continuity,
)


def _header_stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _load_battery(bag, topic):
    msgs = list(bag.messages(topic))
    stamps = np.array([_header_stamp_ns(m) for _, m in msgs], dtype=np.int64)
    voltages = np.array([float(m.voltage) for _, m in msgs], dtype=np.float64)
    return stamps, voltages


def _load_status(bag, topic):
    msgs = list(bag.messages(topic))
    stamps = np.array([_header_stamp_ns(m) for _, m in msgs], dtype=np.int64)
    modes = np.array([int(m.mode) for _, m in msgs], dtype=np.int64)
    voltages = np.array([float(m.battery_voltage) for _, m in msgs], dtype=np.float64)
    return stamps, modes, voltages


def _common_stream_checks(bag, cfg, metrics, pass_fail, thresholds_used):
    """Load and sanity-check both streams; returns the loaded arrays."""
    battery_topic = cfg.get('sensor_nano.topics.battery', '/battery_state')
    status_topic = cfg.get('sensor_nano.topics.mission_status', '/billiebot/mission_status')

    battery_stamps, battery_volts = _load_battery(bag, battery_topic)
    status_stamps, status_modes, status_volts = _load_status(bag, status_topic)

    metrics['battery_topic'] = battery_topic
    metrics['mission_status_topic'] = status_topic
    metrics['battery_message_count'] = int(battery_stamps.size)
    metrics['mission_status_message_count'] = int(status_stamps.size)

    pass_fail['battery_stream_present'] = battery_stamps.size > 0
    pass_fail['mission_status_stream_present'] = status_stamps.size > 0
    thresholds_used['battery_stream_present'] = {
        'value': int(battery_stamps.size), 'threshold': '> 0', 'tier': 'required'}
    thresholds_used['mission_status_stream_present'] = {
        'value': int(status_stamps.size), 'threshold': '> 0', 'tier': 'required'}

    if status_stamps.size:
        max_gap = float(cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_bat_02_max_status_gap_s', 2.0))
        continuity = status_continuity(status_stamps, max_gap)
        metrics['mission_status_continuity'] = continuity
        pass_fail['mission_status_continuous'] = continuity['continuous']
        thresholds_used['mission_status_continuous'] = {
            'value': continuity['max_gap_s'], 'threshold': f'<= {max_gap} s',
            'tier': 'provisional'}

    return battery_stamps, battery_volts, status_stamps, status_modes, status_volts


def _run_physical_profile(bag, cfg, result_dir, args) -> dict:
    metrics = {'profile': 'physical'}
    pass_fail = {}
    thresholds_used = {}

    safe_threshold = float(
        args.safe_threshold_v
        if args.safe_threshold_v is not None
        else cfg.required('sensor_nano.battery.safe_threshold_v')
    )
    high_voltage = float(
        args.high_voltage_v
        if args.high_voltage_v is not None
        else cfg.get('sensor_nano.battery.ut_bat_02_high_voltage_v', 10.70)
    )
    low_voltage = float(
        args.low_voltage_v
        if args.low_voltage_v is not None
        else cfg.get('sensor_nano.battery.ut_bat_02_low_voltage_v', 10.30)
    )
    metrics.update({'safe_threshold_v': safe_threshold, 'high_voltage_v': high_voltage,
                     'low_voltage_v': low_voltage})

    battery_stamps, battery_volts, status_stamps, status_modes, _status_volts = \
        _common_stream_checks(bag, cfg, metrics, pass_fail, thresholds_used)

    if battery_stamps.size == 0 or status_stamps.size == 0:
        return _finalize('UT-BAT-02', metrics, pass_fail, thresholds_used, result_dir)

    first_below_ns = first_crossing_below(battery_stamps, battery_volts, safe_threshold)
    first_safe_ns = first_mode_entry(status_stamps, status_modes, SAFE_MODE)

    metrics['first_below_threshold_ns'] = first_below_ns
    metrics['first_safe_ns'] = first_safe_ns
    metrics['battery_voltage_range_v'] = [float(np.min(battery_volts)),
                                           float(np.max(battery_volts))]
    metrics['modes_observed'] = sorted(int(m) for m in np.unique(status_modes))

    # The battery measurement must actually cross the threshold, otherwise the run never
    # exercised the transition and a SAFE (or a non-SAFE) proves nothing either way.
    pass_fail['battery_crossed_threshold'] = first_below_ns is not None
    thresholds_used['battery_crossed_threshold'] = {
        'value': metrics['battery_voltage_range_v'],
        'threshold': f'at least one sample < {safe_threshold} V', 'tier': 'required'}

    if first_below_ns is not None:
        high_phase = mode_before_time(status_stamps, status_modes, first_below_ns)
        metrics['high_voltage_phase'] = high_phase
        pass_fail['non_safe_at_high_voltage'] = high_phase['stayed_non_safe']
        thresholds_used['non_safe_at_high_voltage'] = {
            'value': high_phase['modes_seen'],
            'threshold': f'no mode {SAFE_MODE} before the first below-threshold sample '
                         f'(held at ~{high_voltage} V)',
            'tier': 'required'}

    pass_fail['safe_transition_occurred'] = first_safe_ns is not None
    thresholds_used['safe_transition_occurred'] = {
        'value': f'mode {SAFE_MODE} seen' if first_safe_ns is not None else 'never observed',
        'threshold': f'mission enters mode {SAFE_MODE} (SAFE)', 'tier': 'required'}

    latency_s = propagation_latency_s(first_below_ns, first_safe_ns)
    metrics['propagation_latency_s'] = latency_s
    max_latency = float(cfg.provisional(
        'sensor_nano.thresholds.provisional.ut_bat_02_max_propagation_latency_s', 2.0))
    # A negative latency means SAFE preceded the trigger -- a pre-existing SAFE state, i.e.
    # a setup fault, not a fast safety chain. Requiring 0 <= latency catches that.
    pass_fail['propagation_latency'] = (
        latency_s is not None and 0.0 <= latency_s <= max_latency
    )
    thresholds_used['propagation_latency'] = {
        'value': latency_s if latency_s is not None else 'not measurable',
        'threshold': f'0 <= latency <= {max_latency} s', 'tier': 'provisional'}

    return _finalize('UT-BAT-02', metrics, pass_fail, thresholds_used, result_dir)


def _load_cases(result_dir) -> list:
    path = result_dir.exports_dir / CASES_CSV
    if not path.exists():
        return []
    with open(path, 'r') as f:
        rows = list(csv.DictReader(f))
    cases = []
    for row in rows:
        try:
            cases.append({
                'case_index': int(row['case_index']),
                'case_voltage_v': float(row['case_voltage_v']),
                'expected_safe': str(row['expected_safe']).strip().lower() == 'true',
                'window_start_ns': int(row['window_start_ns']),
                'window_end_ns': int(row['window_end_ns']),
                'reset_mode_success': str(row.get('reset_mode_success', '')).strip().lower()
                == 'true',
                'pre_case_mode': row.get('pre_case_mode', ''),
            })
        except (KeyError, TypeError, ValueError):
            continue
    return cases


def _run_threshold_profile(bag, cfg, result_dir, args) -> dict:
    metrics = {'profile': 'threshold'}
    pass_fail = {}
    thresholds_used = {}

    safe_threshold = float(
        args.safe_threshold_v
        if args.safe_threshold_v is not None
        else cfg.required('sensor_nano.battery.safe_threshold_v')
    )
    metrics['safe_threshold_v'] = safe_threshold
    metrics['requirement'] = (
        'SYS-PLT-2: enter SAFE at <= 3.5 V/cell, i.e. <= '
        f'{safe_threshold} V for a 3S pack'
    )

    _battery_stamps, _battery_volts, status_stamps, status_modes, _status_volts = \
        _common_stream_checks(bag, cfg, metrics, pass_fail, thresholds_used)

    cases = _load_cases(result_dir)
    metrics['case_count'] = len(cases)

    pass_fail['threshold_cases_recorded'] = len(cases) > 0
    thresholds_used['threshold_cases_recorded'] = {
        'value': len(cases),
        'threshold': f'exports/{CASES_CSV} written by battery_threshold_test',
        'tier': 'required'}

    if not cases or status_stamps.size == 0:
        return _finalize('UT-BAT-02B', metrics, pass_fail, thresholds_used, result_dir)

    results = []
    for case in cases:
        scored = evaluate_threshold_case(
            case['case_voltage_v'], case['expected_safe'],
            status_stamps, status_modes,
            case['window_start_ns'], case['window_end_ns'],
        )
        scored['case_index'] = case['case_index']
        scored['reset_mode_success'] = case['reset_mode_success']
        scored['pre_case_mode'] = case['pre_case_mode']
        results.append(scored)

        name = f"case_{case['case_index']}_{case['case_voltage_v']:.4f}V"
        pass_fail[name] = scored['passed']
        thresholds_used[name] = {
            'value': f"observed SAFE={scored['observed_safe']}",
            'threshold': f"SYS-PLT-2 requires SAFE={scored['expected_safe']}",
            'tier': 'required'}

    metrics['cases'] = results

    boundary = [
        r for r in results
        if abs(r['case_voltage_v'] - safe_threshold) < 1e-9
    ]
    if boundary and not boundary[0]['passed']:
        # This is the expected result until BLK-05 is fixed. It is recorded as an
        # explanation, never as an exemption: the case still counts as a failure.
        metrics['known_blocker'] = (
            'BLK-05: mission_controller.py:147 uses `battery_voltage < battery_safe_voltage` '
            '(strict <), so exactly 10.5000 V does not trigger SAFE. SYS-PLT-2 requires <=. '
            'This UT-BAT-02B failure is the known production requirement discrepancy, not a '
            'Sensor Nano hardware fault, and not a defect in this test.'
        )

    return _finalize('UT-BAT-02B', metrics, pass_fail, thresholds_used, result_dir)


def _finalize(test_id, metrics, pass_fail, thresholds_used, result_dir) -> dict:
    overall = all(v for k, v in pass_fail.items() if k != 'overall')
    pass_fail['overall'] = overall
    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md(test_id, metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    """Offline check of the SYS-PLT-2 expectation table and the latency computation."""
    from billiebot_sensor_tests.sensor_nano.safety_metrics import requirement_expects_safe

    expectations = [(10.5001, False), (10.5000, True), (10.4999, True)]
    ok = all(requirement_expects_safe(v, 10.5) is expected for v, expected in expectations)
    for voltage, expected in expectations:
        print(f'[self-test] {voltage:.4f} V -> SYS-PLT-2 requires SAFE={expected}')
    latency = propagation_latency_s(1_000_000_000, 2_500_000_000)
    print(f'[self-test] propagation latency = {latency:.2f} s (expected 1.50)')
    ok = ok and abs(latency - 1.5) < 1e-9
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Score low-battery SAFE propagation (UT-BAT-02 / UT-BAT-02B).'
    )
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    parser.add_argument('--profile', choices=['physical', 'threshold'], default='physical')
    parser.add_argument('--high-voltage-v', type=float, default=None,
                         help='operator-held high setpoint; defaults to sensor_bench.yaml')
    parser.add_argument('--low-voltage-v', type=float, default=None,
                         help='operator-held low setpoint; defaults to sensor_bench.yaml')
    parser.add_argument('--safe-threshold-v', type=float, default=None,
                         help='SAFE threshold under test; defaults to sensor_bench.yaml')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args(argv)

    if args.self_test:
        return _self_test()

    if not args.results_dir:
        parser.error('--results-dir is required unless --self-test is given')

    result_dir = BenchResultDir(args.results_dir)
    cfg = load_bench_config(args.config_file or _default_config_path())
    bag = BagReader(result_dir.bag_dir)

    if bag.is_empty:
        write_metrics_json(
            result_dir.metrics_json_path, {'error': 'bag is empty or missing'},
            {'overall': False}
        )
        print('ERROR: bag is empty or missing', file=sys.stderr)
        return 1

    if args.profile == 'physical':
        result = _run_physical_profile(bag, cfg, result_dir, args)
    else:
        result = _run_threshold_profile(bag, cfg, result_dir, args)

    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
