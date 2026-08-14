#!/usr/bin/env python3
"""CLI: UT-BAT-01 battery-divider acquisition and voltage accuracy.

Two evidence sources, deliberately independent:

  * the rosbag -- authoritative /battery_state and /bench/battery/adc time series, used for
    publication rate, gaps and message integrity;
  * exports/battery_points.csv -- the operator's DMM ground truth per PSU setpoint, written
    by `record_battery_point`, used for every accuracy figure.

Accuracy is always ROS-versus-DMM. Nothing here re-derives `battery_divider_ratio` or
`adc_reference_voltage` and feeds it back: `fit_observed_scale()` reports what the hardware
implies so a human can decide whether to change configuration, but the verdict is computed
against the values the run actually used. Auto-calibrating would turn a miswired divider
into a PASS.
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats
from billiebot_sensor_tests.common.report import (
    render_report_md, write_metrics_csv, write_metrics_json,
)
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.sensor_nano.battery_metrics import (
    ADC_MAX_COUNT,
    adc_monotonicity,
    adc_range_check,
    error_near_threshold,
    error_stats,
    fit_observed_scale,
    linear_fit,
    measured_divider_ratio,
)
from billiebot_sensor_tests.sensor_nano.battery_point_recorder import CSV_NAME


def _header_stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _load_points(result_dir) -> list:
    """Read exports/battery_points.csv, keeping only rows that carry usable DMM ground truth."""
    path = result_dir.exports_dir / CSV_NAME
    if not path.exists():
        return []
    with open(path, 'r') as f:
        rows = list(csv.DictReader(f))

    points = []
    for row in rows:
        try:
            point = {
                'setpoint_v': float(row['setpoint_v']),
                'dmm_battery_v': float(row['dmm_battery_v']),
                'dmm_a0_v': float(row['dmm_a0_v']),
            }
        except (KeyError, TypeError, ValueError):
            continue
        for key in ('ros_voltage_mean_v', 'adc_mean', 'ros_voltage_std_v', 'adc_std'):
            try:
                point[key] = float(row[key])
            except (KeyError, TypeError, ValueError):
                point[key] = float('nan')
        for key in ('ros_voltage_count', 'adc_count'):
            try:
                point[key] = int(row[key])
            except (KeyError, TypeError, ValueError):
                point[key] = 0
        point['notes'] = row.get('notes', '')
        points.append(point)
    return points


def _write_plots(result_dir, points, metrics) -> list:
    """Render the UT-BAT-01 plots. Returns the list of written paths (possibly empty).

    matplotlib is imported here, lazily and defensively: plots are non-authoritative
    visualization, so a host without matplotlib must still produce a valid verdict from the
    bag and the CSV rather than failing the analysis.
    """
    if not points:
        return []
    try:
        import matplotlib
        matplotlib.use('Agg')  # headless: analysis often runs over SSH on the Jetson
        import matplotlib.pyplot as plt
    except ImportError:
        return []

    plots_dir = result_dir.plots_dir
    plots_dir.mkdir(parents=True, exist_ok=True)

    dmm = np.array([p['dmm_battery_v'] for p in points])
    ros = np.array([p['ros_voltage_mean_v'] for p in points])
    a0 = np.array([p['dmm_a0_v'] for p in points])
    adc = np.array([p['adc_mean'] for p in points])
    errors = ros - dmm
    order = np.argsort(dmm)

    written = []

    def _save(fig, name):
        path = plots_dir / name
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        written.append(str(path))

    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.plot(dmm[order], ros[order], 'o-', label='ROS /battery_state')
    limits = [float(np.nanmin(dmm)), float(np.nanmax(dmm))]
    ax.plot(limits, limits, 'k--', linewidth=1, label='ideal (y = x)')
    ax.set_xlabel('DMM battery voltage (V)')
    ax.set_ylabel('ROS reported voltage (V)')
    ax.set_title('UT-BAT-01: ROS voltage vs DMM')
    ax.grid(True, alpha=0.3)
    ax.legend()
    _save(fig, 'ros_voltage_vs_dmm.png')

    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.axhline(0.0, color='k', linewidth=1)
    ax.plot(dmm[order], errors[order], 'o-')
    threshold = metrics.get('safe_threshold_v')
    if threshold is not None:
        ax.axvline(threshold, color='r', linestyle=':',
                   label=f'SAFE threshold {threshold} V')
        ax.legend()
    ax.set_xlabel('DMM battery voltage (V)')
    ax.set_ylabel('ROS - DMM error (V)')
    ax.set_title('UT-BAT-01: voltage error vs DMM')
    ax.grid(True, alpha=0.3)
    _save(fig, 'voltage_error_vs_dmm.png')

    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.plot(dmm[order], adc[order], 'o-')
    ax.set_xlabel('DMM battery voltage (V)')
    ax.set_ylabel('mean raw ADC count')
    ax.set_title('UT-BAT-01: ADC count vs input voltage')
    ax.grid(True, alpha=0.3)
    _save(fig, 'adc_vs_dmm.png')

    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.plot(dmm[order], a0[order], 'o-')
    ax.set_xlabel('DMM battery voltage (V)')
    ax.set_ylabel('DMM divider-node voltage A0 (V)')
    ax.set_title('UT-BAT-01: divider node vs battery voltage')
    ax.grid(True, alpha=0.3)
    _save(fig, 'divider_node_vs_battery.png')

    return written


def _bag_series(bag, cfg):
    """(battery stamps/voltages, adc values) straight from the bag."""
    battery_topic = cfg.get('sensor_nano.topics.battery', '/battery_state')
    adc_topic = cfg.get('sensor_nano.topics.battery_adc', '/bench/battery/adc')

    battery_msgs = list(bag.messages(battery_topic))
    stamps = np.array([_header_stamp_ns(m) for _, m in battery_msgs], dtype=np.int64)
    voltages = np.array([float(m.voltage) for _, m in battery_msgs], dtype=np.float64)

    adc_msgs = list(bag.messages(adc_topic))
    adc_values = np.array([float(m.data) for _, m in adc_msgs], dtype=np.float64)

    return stamps, voltages, adc_values, battery_topic, adc_topic


def _load_parser_stats(result_dir, bag, diagnostics_topic: str) -> dict:
    export_path = result_dir.exports_dir / 'sensor_nano_parser_stats.json'
    if export_path.exists():
        try:
            data = json.loads(export_path.read_text())
            data['source'] = 'exports/sensor_nano_parser_stats.json'
            return data
        except (OSError, ValueError):
            pass
    messages = list(bag.messages(diagnostics_topic))
    if not messages:
        return {'source': 'unavailable'}
    _stamp, last = messages[-1]
    data = {'source': f'{diagnostics_topic} (last sample)'}
    for status in last.status:
        for entry in status.values:
            try:
                data[entry.key] = json.loads(entry.value)
            except ValueError:
                data[entry.key] = entry.value
    return data


def _run(bag, cfg, result_dir) -> dict:
    metrics = {}
    pass_fail = {}
    thresholds_used = {}

    configured_ratio = float(cfg.get('sensor_nano.battery.divider_ratio', 6.0))
    adc_reference = float(cfg.get('sensor_nano.battery.adc_reference_voltage', 5.0))
    safe_threshold = float(cfg.required('sensor_nano.battery.safe_threshold_v'))
    metrics['configured_divider_ratio'] = configured_ratio
    metrics['configured_adc_reference_voltage'] = adc_reference
    metrics['safe_threshold_v'] = safe_threshold

    # -- bag-derived stream health ------------------------------------------------------
    stamps, voltages, adc_values, battery_topic, adc_topic = _bag_series(bag, cfg)
    metrics['battery_topic'] = battery_topic
    metrics['battery_message_count'] = int(stamps.size)
    metrics['adc_topic'] = adc_topic
    metrics['adc_message_count'] = int(adc_values.size)
    metrics['battery_message_type'] = bag.topic_names_and_types.get(battery_topic, '')

    pass_fail['battery_stream_present'] = stamps.size > 0
    thresholds_used['battery_stream_present'] = {
        'value': int(stamps.size), 'threshold': '> 0', 'tier': 'required'}

    if stamps.size:
        rate = compute_rate_stats(stamps)
        metrics['battery_rate'] = rate.to_dict()
        min_rate = cfg.required('sensor_nano.thresholds.required.ut_bat_01_min_battery_rate_hz')
        max_gap = cfg.required('sensor_nano.thresholds.required.ut_bat_01_max_battery_gap_s')
        pass_fail['battery_rate'] = rate.mean_hz >= min_rate
        pass_fail['battery_max_gap'] = rate.max_gap_s <= max_gap
        thresholds_used['battery_rate'] = {
            'value': rate.mean_hz, 'threshold': f'>= {min_rate} Hz', 'tier': 'required'}
        thresholds_used['battery_max_gap'] = {
            'value': rate.max_gap_s, 'threshold': f'<= {max_gap} s', 'tier': 'required'}

    if adc_values.size:
        adc_range = adc_range_check(adc_values, ADC_MAX_COUNT)
        metrics['adc_range'] = adc_range
        pass_fail['adc_not_saturated'] = not (
            adc_range['saturated_low'] or adc_range['saturated_high']
        )
        thresholds_used['adc_not_saturated'] = {
            'value': f"{adc_range['min_count']}-{adc_range['max_count']}",
            'threshold': f'strictly inside 0-{ADC_MAX_COUNT:g}', 'tier': 'required'}

    parser = _load_parser_stats(
        result_dir, bag,
        cfg.get('sensor_nano.topics.diagnostics', '/bench/sensor_nano/diagnostics')
    )
    metrics['parser_stats'] = parser
    max_parse_error = cfg.required(
        'sensor_nano.thresholds.required.ut_bat_01_max_parse_error_fraction'
    )
    try:
        parse_fraction = float(parser.get('parse_error_fraction'))
    except (TypeError, ValueError):
        parse_fraction = None
    metrics['parse_error_fraction'] = parse_fraction
    pass_fail['parse_error_fraction'] = (
        parse_fraction is not None and parse_fraction <= max_parse_error
    )
    thresholds_used['parse_error_fraction'] = {
        'value': parse_fraction if parse_fraction is not None else 'unavailable',
        'threshold': f'<= {max_parse_error}', 'tier': 'required'}

    # -- operator ground-truth accuracy -------------------------------------------------
    points = _load_points(result_dir)
    metrics['calibration_point_count'] = len(points)
    metrics['calibration_points'] = points

    min_points = int(cfg.provisional(
        'sensor_nano.thresholds.provisional.ut_bat_01_min_calibration_points', 5))
    pass_fail['enough_calibration_points'] = len(points) >= min_points
    thresholds_used['enough_calibration_points'] = {
        'value': len(points), 'threshold': f'>= {min_points}', 'tier': 'provisional'}

    if points:
        dmm_bat = np.array([p['dmm_battery_v'] for p in points])
        dmm_a0 = np.array([p['dmm_a0_v'] for p in points])
        ros_v = np.array([p['ros_voltage_mean_v'] for p in points])
        adc_mean = np.array([p['adc_mean'] for p in points])

        ratios = measured_divider_ratio(dmm_bat, dmm_a0)
        finite_ratios = ratios[np.isfinite(ratios)]
        mean_ratio = float(np.mean(finite_ratios)) if finite_ratios.size else float('nan')
        metrics['measured_divider_ratio'] = {
            'per_point': [float(r) for r in ratios],
            'mean': mean_ratio,
            'std': float(np.std(finite_ratios)) if finite_ratios.size else float('nan'),
            'configured': configured_ratio,
            'relative_error': (
                float(abs(mean_ratio - configured_ratio) / configured_ratio)
                if np.isfinite(mean_ratio) and configured_ratio else float('nan')
            ),
        }
        max_ratio_error = cfg.required(
            'sensor_nano.thresholds.required.ut_bat_01_max_divider_ratio_relative_error'
        )
        ratio_error = metrics['measured_divider_ratio']['relative_error']
        pass_fail['divider_ratio_matches_configuration'] = (
            np.isfinite(ratio_error) and ratio_error <= max_ratio_error
        )
        thresholds_used['divider_ratio_matches_configuration'] = {
            'value': ratio_error, 'threshold': f'<= {max_ratio_error}', 'tier': 'required'}

        metrics['adc_monotonicity'] = adc_monotonicity(
            dmm_bat, adc_mean,
            float(cfg.provisional(
                'sensor_nano.thresholds.provisional.ut_bat_01_adc_monotonicity_tolerance_counts',
                2.0)),
        )
        pass_fail['adc_monotonic'] = metrics['adc_monotonicity']['monotonic']
        thresholds_used['adc_monotonic'] = {
            'value': metrics['adc_monotonicity']['max_decrease_counts'],
            'threshold': 'non-decreasing with increasing input voltage', 'tier': 'required'}

        metrics['voltage_linear_fit'] = linear_fit(dmm_bat, ros_v)
        metrics['observed_scale_reporting_only'] = fit_observed_scale(
            dmm_bat, adc_mean, ADC_MAX_COUNT
        )
        metrics['voltage_error'] = error_stats(ros_v, dmm_bat)

        max_abs_error = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_bat_01_max_abs_error_v', 0.20)
        max_rmse = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_bat_01_max_rmse_v', 0.15)
        pass_fail['voltage_max_abs_error'] = (
            np.isfinite(metrics['voltage_error']['max_abs_error_v'])
            and metrics['voltage_error']['max_abs_error_v'] <= max_abs_error
        )
        pass_fail['voltage_rmse'] = (
            np.isfinite(metrics['voltage_error']['rmse_v'])
            and metrics['voltage_error']['rmse_v'] <= max_rmse
        )
        thresholds_used['voltage_max_abs_error'] = {
            'value': metrics['voltage_error']['max_abs_error_v'],
            'threshold': f'<= {max_abs_error} V', 'tier': 'provisional'}
        thresholds_used['voltage_rmse'] = {
            'value': metrics['voltage_error']['rmse_v'],
            'threshold': f'<= {max_rmse} V', 'tier': 'provisional'}

        window = float(cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_bat_01_threshold_window_v', 0.30))
        near = error_near_threshold(dmm_bat, ros_v, safe_threshold, window)
        metrics['error_near_safe_threshold'] = near
        max_threshold_error = cfg.required(
            'sensor_nano.thresholds.required.ut_bat_01_max_threshold_region_error_v'
        )
        # No points near 10.5 V means the safety-critical region was never characterized,
        # which fails the criterion rather than passing it by default.
        pass_fail['error_near_safe_threshold'] = (
            near['n_points'] > 0
            and np.isfinite(near['max_abs_error_v'])
            and near['max_abs_error_v'] <= max_threshold_error
        )
        thresholds_used['error_near_safe_threshold'] = {
            'value': near['max_abs_error_v'] if near['n_points'] else 'no points in window',
            'threshold': f'<= {max_threshold_error} V within +/-{window} V of '
                         f'{safe_threshold} V',
            'tier': 'required'}

        min_r_squared = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_bat_01_min_r_squared', 0.999)
        r_squared = metrics['voltage_linear_fit']['r_squared']
        pass_fail['no_gross_nonlinearity'] = (
            np.isfinite(r_squared) and r_squared >= min_r_squared
        )
        thresholds_used['no_gross_nonlinearity'] = {
            'value': r_squared, 'threshold': f'r^2 >= {min_r_squared}', 'tier': 'provisional'}

    plots = _write_plots(result_dir, points, metrics)
    metrics['plots'] = [os.path.basename(p) for p in plots]

    overall = all(v for k, v in pass_fail.items() if k != 'overall')
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-BAT-01', metrics, thresholds_used, pass_fail, plots=plots)
    )
    return {'pass': overall}


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    """Offline check that a known-good synthetic sweep scores as accurate and monotonic."""
    from billiebot_sensor_tests.sensor_nano.battery_metrics import adc_to_volts

    dmm = np.array([9.9, 10.3, 10.5, 10.7, 11.1, 12.0, 12.6])
    adc = np.round(dmm / 6.0 / 5.0 * ADC_MAX_COUNT)
    ros = adc_to_volts(adc, 5.0, 6.0)
    stats = error_stats(ros, dmm)
    mono = adc_monotonicity(dmm, adc)
    print(f"[self-test] synthetic sweep max|err| = {stats['max_abs_error_v']:.4f} V, "
          f"monotonic = {mono['monotonic']}")
    ok = stats['max_abs_error_v'] <= 0.05 and mono['monotonic']
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Score a UT-BAT-01 battery divider acquisition/accuracy run.'
    )
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    # Accepted and ignored so the standard orchestrator, which always forwards the test
    # spec's profile, can drive this analyzer unchanged.
    parser.add_argument('--profile', default=None, help=argparse.SUPPRESS)
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

    result = _run(bag, cfg, result_dir)
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
