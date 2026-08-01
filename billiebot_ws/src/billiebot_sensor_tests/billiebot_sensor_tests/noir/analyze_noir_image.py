#!/usr/bin/env python3
"""CLI: UT-NIR-01 stream check (--profile stream) and UT-NIR-02 focus/quality/low-light
(--profile quality, combines multiple --capture-dir runs -- one per lighting condition).
Reads recorded rosbag2 data offline via BagReader."""

import argparse
import os
import sys
from pathlib import Path

import numpy as np

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.image_hash import repeated_frame_hash
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats
from billiebot_sensor_tests.common.report import render_report_md, write_metrics_csv, write_metrics_json
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.noir.metrics import (
    clipping_fraction,
    contrast_to_noise_ratio,
    laplacian_variance,
    temporal_stability,
)


def _header_stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _load_image(msg) -> np.ndarray:
    channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return arr[:msg.width * msg.height * channels].reshape(msg.height, msg.width, channels)


def _run_stream_profile(bag, cfg, result_dir) -> dict:
    image_topic = cfg.get('noir.topics.image', '/noir/image')
    msgs = list(bag.messages(image_topic))

    metrics = {'frame_count': len(msgs)}
    pass_fail = {'image_present': len(msgs) > 0}
    thresholds_used = {
        'image_present': {'value': len(msgs) > 0, 'threshold': True, 'tier': 'required'},
    }

    if msgs:
        expected_w = int(cfg.get('noir.dimensions.width_px', 640))
        expected_h = int(cfg.get('noir.dimensions.height_px', 480))
        first = msgs[0][1]
        pass_fail['dimensions_correct'] = (first.width == expected_w and first.height == expected_h)
        pass_fail['encoding_correct'] = (first.encoding == cfg.get('noir.encodings.image', 'rgb8'))

        stats = compute_rate_stats([_header_stamp_ns(m) for _, m in msgs])
        metrics['rate'] = stats.to_dict()
        min_rate = cfg.required('noir.thresholds.required.ut_nir_01_min_rate_hz')
        max_gap = cfg.required('noir.thresholds.required.ut_nir_01_max_gap_s')
        pass_fail['min_rate'] = stats.mean_hz >= min_rate
        pass_fail['max_gap'] = stats.max_gap_s <= max_gap
        thresholds_used['min_rate'] = {'value': stats.mean_hz, 'threshold': min_rate, 'tier': 'required'}
        thresholds_used['max_gap'] = {'value': stats.max_gap_s, 'threshold': max_gap, 'tier': 'required'}

        channels = 3 if first.encoding in ('rgb8', 'bgr8') else 1
        hashes = [
            repeated_frame_hash(bytes(m.data), m.width, m.height, channels) for _, m in msgs
        ]
        repeated = sum(1 for i in range(1, len(hashes)) if hashes[i] == hashes[i - 1])
        repeat_fraction = (repeated / len(hashes)) if hashes else 0.0
        metrics['repeated_frame_fraction'] = repeat_fraction
        max_repeat = cfg.required('noir.thresholds.required.ut_nir_01_max_repeat_dropped_fraction')
        pass_fail['repeat_fraction_within_limit'] = repeat_fraction <= max_repeat
        thresholds_used['repeat_fraction_within_limit'] = {
            'value': repeat_fraction, 'threshold': max_repeat, 'tier': 'required'}

    overall = all(pass_fail.values())
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-NIR-01', metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _analyze_capture_dir(capture_dir, cfg) -> dict:
    result_dir = BenchResultDir(capture_dir)
    bag = BagReader(result_dir.bag_dir)
    image_topic = cfg.get('noir.topics.image', '/noir/image')
    msgs = list(bag.messages(image_topic))
    if not msgs:
        return None

    grays = [_load_image(m).mean(axis=2) for _, m in msgs]
    width = grays[0].shape[1]
    # Default ROIs when the operator hasn't saved an explicit one: outer thirds of the
    # frame as a rough black/white patch stand-in for the printed test chart.
    black_vals = np.concatenate([g[:, : width // 3].flatten() for g in grays])
    white_vals = np.concatenate([g[:, -width // 3:].flatten() for g in grays])

    cnr = contrast_to_noise_ratio(white_vals, black_vals)
    sharpness_values = [laplacian_variance(g) for g in grays]
    brightness_values = [float(np.mean(g)) for g in grays]
    clip_values = [clipping_fraction(g, 0, 255) for g in grays]

    return {
        'n_frames': len(grays),
        'cnr': cnr,
        'mean_sharpness': float(np.mean(sharpness_values)),
        'sharpness_cov': temporal_stability(sharpness_values),
        'brightness_cov': temporal_stability(brightness_values),
        'mean_clipping_fraction': float(np.mean(clip_values)),
    }


def _run_quality_profile(capture_dirs, cfg, result_dir, condition_labels) -> dict:
    per_condition = {}
    for path, label in zip(capture_dirs, condition_labels):
        result = _analyze_capture_dir(path, cfg)
        if result is not None:
            per_condition[label] = result

    metrics = {'conditions': per_condition}
    pass_fail = {}
    thresholds_used = {}

    normal = per_condition.get('normal_light')
    if normal:
        min_cnr = cfg.provisional('noir.thresholds.provisional.ut_nir_02_normal_min_cnr', 10.0)
        max_clip = cfg.required('noir.thresholds.required.ut_nir_02_normal_max_clipping_fraction')
        max_cov = cfg.provisional(
            'noir.thresholds.provisional.ut_nir_02_normal_max_sharpness_cov', 0.20
        )
        pass_fail['normal_cnr_within_provisional_limit'] = normal['cnr'] >= min_cnr
        pass_fail['normal_clipping_within_limit'] = normal['mean_clipping_fraction'] <= max_clip
        pass_fail['normal_sharpness_stable'] = normal['sharpness_cov'] <= max_cov
        thresholds_used['normal_cnr_within_provisional_limit'] = {
            'value': normal['cnr'], 'threshold': min_cnr, 'tier': 'provisional'}
        thresholds_used['normal_clipping_within_limit'] = {
            'value': normal['mean_clipping_fraction'], 'threshold': max_clip, 'tier': 'required'}
        thresholds_used['normal_sharpness_stable'] = {
            'value': normal['sharpness_cov'], 'threshold': max_cov, 'tier': 'provisional'}

    ir_dark = per_condition.get('dark_with_ir')
    if ir_dark:
        min_ir_cnr = cfg.provisional('noir.thresholds.provisional.ut_nir_02_ir_dark_min_cnr', 5.0)
        pass_fail['ir_dark_cnr_within_provisional_limit'] = ir_dark['cnr'] >= min_ir_cnr
        thresholds_used['ir_dark_cnr_within_provisional_limit'] = {
            'value': ir_dark['cnr'], 'threshold': min_ir_cnr, 'tier': 'provisional'}

    # dark_no_ir is explicitly characterization-only per spec -- it never gates pass/fail.

    required_checks = [
        v for k, v in pass_fail.items()
        if thresholds_used.get(k, {}).get('tier') == 'required'
    ]
    required_pass = all(required_checks) if required_checks else bool(per_condition)
    pass_fail['overall'] = required_pass

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-NIR-02', metrics, thresholds_used, pass_fail)
    )
    return {'pass': required_pass}


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    rng = np.random.default_rng(0)
    white = 200 + rng.normal(0, 3, size=500)
    black = 30 + rng.normal(0, 3, size=500)
    cnr = contrast_to_noise_ratio(white, black)
    print(f'[self-test] synthetic white/black CNR = {cnr:.2f}')
    ok = cnr >= 10.0
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    parser.add_argument('--profile', choices=['stream', 'quality'], default='stream')
    parser.add_argument('--capture-dir', action='append', default=[],
                         help='One per lighting condition, for --profile quality')
    parser.add_argument('--condition', action='append', default=[],
                         help='Label matching each --capture-dir, e.g. normal_light, dim, '
                              'dark_no_ir, dark_with_ir')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args(argv)

    if args.self_test:
        return _self_test()

    if not args.results_dir:
        parser.error('--results-dir is required unless --self-test is given')

    cfg = load_bench_config(args.config_file or _default_config_path())

    if args.profile == 'stream':
        result_dir = BenchResultDir(args.results_dir)
        bag = BagReader(result_dir.bag_dir)
        if bag.is_empty:
            write_metrics_json(
                result_dir.metrics_json_path, {'error': 'bag is empty or missing'}, {'overall': False}
            )
            print('ERROR: bag is empty or missing', file=sys.stderr)
            return 1
        result = _run_stream_profile(bag, cfg, result_dir)
    else:
        if not args.capture_dir:
            parser.error('--profile quality requires at least one --capture-dir')
        labels = args.condition or [f'condition_{i}' for i in range(len(args.capture_dir))]
        Path(args.results_dir).mkdir(parents=True, exist_ok=True)
        result_dir = BenchResultDir(args.results_dir)
        result = _run_quality_profile(args.capture_dir, cfg, result_dir, labels)

    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
