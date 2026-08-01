#!/usr/bin/env python3
"""CLI: UT-OAK-01 stream check (--profile stream) and UT-OAK-02 2.0 m depth accuracy
(--profile accuracy). Reads recorded rosbag2 data offline via BagReader -- never requires
a live ROS node, so this can run on a copied bag on another machine."""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats
from billiebot_sensor_tests.common.report import render_report_md, write_metrics_csv, write_metrics_json
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.oakd.metrics import (
    fit_plane_svd,
    fraction_within_band,
    percentiles,
    plane_normal_angle_deg,
    point_to_plane_rmse,
    robust_std_mad,
    valid_pixel_fraction,
)


def _header_stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _run_stream_profile(bag, cfg, result_dir) -> dict:
    rgb_topic = cfg.get('oakd.topics.rgb', '/bench/oakd/rgb/image_raw')
    depth_topic = cfg.get('oakd.topics.depth', '/bench/oakd/depth/image_raw')
    points_topic = cfg.get('oakd.topics.points', '/bench/oakd/points')

    rgb_msgs = list(bag.messages(rgb_topic))
    depth_msgs = list(bag.messages(depth_topic))
    points_msgs = list(bag.messages(points_topic))

    metrics = {
        'rgb_count': len(rgb_msgs),
        'depth_count': len(depth_msgs),
        'points_count': len(points_msgs),
    }
    pass_fail = {}
    thresholds_used = {}

    for name, msgs in (('rgb', rgb_msgs), ('depth', depth_msgs), ('points', points_msgs)):
        present = len(msgs) > 0
        pass_fail[f'{name}_present'] = present
        thresholds_used[f'{name}_present'] = {'value': present, 'threshold': True, 'tier': 'required'}

    min_rate = cfg.required('oakd.thresholds.required.ut_oak_01_min_avg_rate_hz')
    max_gap = cfg.required('oakd.thresholds.required.ut_oak_01_max_gap_s')

    for name, msgs in (('rgb', rgb_msgs), ('depth', depth_msgs)):
        if not msgs:
            continue
        stats = compute_rate_stats([_header_stamp_ns(m) for _, m in msgs])
        metrics[f'{name}_rate'] = stats.to_dict()
        pass_fail[f'{name}_min_rate'] = stats.mean_hz >= min_rate
        pass_fail[f'{name}_max_gap'] = stats.max_gap_s <= max_gap
        pass_fail[f'{name}_monotonic'] = stats.monotonic
        thresholds_used[f'{name}_min_rate'] = {
            'value': stats.mean_hz, 'threshold': min_rate, 'tier': 'required'}
        thresholds_used[f'{name}_max_gap'] = {
            'value': stats.max_gap_s, 'threshold': max_gap, 'tier': 'required'}

    overall = all(pass_fail.values()) if pass_fail else False
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-OAK-01', metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _load_depth_frame(depth_msg) -> np.ndarray:
    arr = np.frombuffer(bytes(depth_msg.data), dtype=np.uint16).reshape(
        depth_msg.height, depth_msg.width
    )
    return arr.astype(np.float64) / 1000.0  # mm -> m


def _load_roi_mask(shape, roi_path):
    if roi_path is None or not Path(roi_path).exists():
        return np.ones(shape, dtype=bool)
    with open(roi_path, 'r') as f:
        roi = json.load(f)
    mask = np.zeros(shape, dtype=bool)
    x0 = roi.get('x0', 0)
    y0 = roi.get('y0', 0)
    x1 = roi.get('x1', shape[1])
    y1 = roi.get('y1', shape[0])
    mask[y0:y1, x0:x1] = True
    return mask


def _run_accuracy_profile(bag, cfg, result_dir, ground_truth_m, roi_path) -> dict:
    depth_topic = cfg.get('oakd.topics.depth', '/bench/oakd/depth/image_raw')
    depth_msgs = list(bag.messages(depth_topic))
    if not depth_msgs:
        write_metrics_json(
            result_dir.metrics_json_path, {'error': 'no depth frames recorded'}, {'overall': False}
        )
        return {'pass': False}

    frames = [_load_depth_frame(m) for _, m in depth_msgs]
    mask = _load_roi_mask(frames[0].shape, roi_path)
    roi_values = np.concatenate([f[mask] for f in frames])

    min_m = cfg.get('oakd.depth_limits.min_m', 0.1)
    max_m = cfg.get('oakd.depth_limits.max_m', 5.0)
    valid_frac = valid_pixel_fraction(roi_values, min_m, max_m)
    finite = roi_values[np.isfinite(roi_values) & (roi_values >= min_m) & (roi_values <= max_m)]

    median_range = float(np.median(finite)) if finite.size else float('nan')
    bias = median_range - ground_truth_m
    robust_std = robust_std_mad(finite)
    pct = percentiles(finite)
    band_m = cfg.required('oakd.thresholds.required.ut_oak_02_band_m')
    within_band = fraction_within_band(finite, ground_truth_m, band_m)

    xs, ys = np.meshgrid(np.arange(frames[0].shape[1]), np.arange(frames[0].shape[0]))
    points_xyz = np.stack(
        [xs[mask].astype(np.float64), ys[mask].astype(np.float64), frames[0][mask]], axis=1
    )
    points_xyz = points_xyz[np.isfinite(points_xyz[:, 2]) & (points_xyz[:, 2] > 0)]
    plane_rmse = float('nan')
    normal_angle = float('nan')
    if points_xyz.shape[0] >= 3:
        normal, centroid = fit_plane_svd(points_xyz)
        plane_rmse = point_to_plane_rmse(points_xyz, normal, centroid)
        normal_angle = plane_normal_angle_deg(normal)

    metrics = {
        'ground_truth_m': ground_truth_m,
        'median_range_m': median_range,
        'mean_range_m': float(np.mean(finite)) if finite.size else float('nan'),
        'bias_m': bias,
        'robust_std_m': robust_std,
        'std_m': float(np.std(finite)) if finite.size else float('nan'),
        'percentiles_m': pct,
        'valid_pixel_fraction': valid_frac,
        'fraction_within_band': within_band,
        'plane_rmse_m': plane_rmse,
        'plane_normal_angle_deg': normal_angle,
    }

    max_bias = cfg.required('oakd.thresholds.required.ut_oak_02_max_median_bias_m')
    min_valid = cfg.required('oakd.thresholds.required.ut_oak_02_min_valid_fraction')
    min_within = cfg.required('oakd.thresholds.required.ut_oak_02_min_within_band_fraction')
    plane_rmse_limit = cfg.provisional('oakd.thresholds.provisional.ut_oak_02_plane_rmse_m', 0.05)

    pass_fail = {
        'median_bias_within_limit': abs(bias) <= max_bias,
        'valid_fraction_sufficient': valid_frac >= min_valid,
        'within_band_fraction_sufficient': within_band >= min_within,
        'plane_rmse_within_provisional_limit': (
            plane_rmse <= plane_rmse_limit if plane_rmse == plane_rmse else False
        ),
    }
    thresholds_used = {
        'median_bias_within_limit': {'value': bias, 'threshold': max_bias, 'tier': 'required'},
        'valid_fraction_sufficient': {
            'value': valid_frac, 'threshold': min_valid, 'tier': 'required'},
        'within_band_fraction_sufficient': {
            'value': within_band, 'threshold': min_within, 'tier': 'required'},
        'plane_rmse_within_provisional_limit': {
            'value': plane_rmse, 'threshold': plane_rmse_limit, 'tier': 'provisional'},
    }
    # Provisional plane-RMSE never fails the run by itself -- only the three required checks do.
    required_pass = (
        pass_fail['median_bias_within_limit']
        and pass_fail['valid_fraction_sufficient']
        and pass_fail['within_band_fraction_sufficient']
    )
    pass_fail['overall'] = required_pass

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-OAK-02', metrics, thresholds_used, pass_fail)
    )
    return {'pass': required_pass}


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    """Exercises the accuracy-profile math against inline synthetic data. Works from an
    installed console_script with no dependency on the package's test/fixtures/ source tree
    (which is deliberately excluded from the installed package)."""
    rng = np.random.default_rng(0)
    xs, ys = np.meshgrid(np.linspace(-0.3, 0.3, 40), np.linspace(-0.3, 0.3, 40))
    zz = 2.0 + rng.normal(0.0, 0.01, size=xs.shape)
    points = np.stack([xs.ravel(), ys.ravel(), zz.ravel()], axis=1)
    normal, centroid = fit_plane_svd(points)
    rmse = point_to_plane_rmse(points, normal, centroid)
    print(f'[self-test] synthetic 2.0m plane: plane RMSE = {rmse:.4f} m')
    ok = rmse < 0.05
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    parser.add_argument('--profile', choices=['stream', 'accuracy'], default='stream')
    parser.add_argument('--ground-truth-m', type=float, default=2.0)
    parser.add_argument('--roi', default=None)
    parser.add_argument('--self-test', action='store_true',
                         help='Run against inline synthetic data, no bag/hardware required.')
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
            result_dir.metrics_json_path, {'error': 'bag is empty or missing'}, {'overall': False}
        )
        print('ERROR: bag is empty or missing', file=sys.stderr)
        return 1

    if args.profile == 'stream':
        result = _run_stream_profile(bag, cfg, result_dir)
    else:
        result = _run_accuracy_profile(bag, cfg, result_dir, args.ground_truth_m, args.roi)

    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
