#!/usr/bin/env python3
"""CLI: UT-THM-01 stream check (--profile stream) and UT-THM-02 target/background
contrast (--profile contrast). Reads recorded rosbag2 data offline via BagReader."""

import argparse
import os
import sys

import numpy as np

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats
from billiebot_sensor_tests.common.report import render_report_md, write_metrics_csv, write_metrics_json
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.thermal.metrics import (
    contrast_to_noise_ratio,
    drift,
    pooled_cnr,
    target_background_bias,
    temporal_noise,
)


def _header_stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _load_frame(msg) -> np.ndarray:
    return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)


def _run_stream_profile(bag, cfg, result_dir) -> dict:
    image_topic = cfg.get('thermal.topics.image', '/thermal/image')
    msgs = list(bag.messages(image_topic))

    metrics = {'frame_count': len(msgs)}
    pass_fail = {'image_present': len(msgs) > 0}
    thresholds_used = {
        'image_present': {'value': len(msgs) > 0, 'threshold': True, 'tier': 'required'},
    }

    if msgs:
        expected_w = int(cfg.get('thermal.dimensions.width_px', 32))
        expected_h = int(cfg.get('thermal.dimensions.height_px', 24))
        first = msgs[0][1]
        pass_fail['dimensions_correct'] = (first.width == expected_w and first.height == expected_h)
        pass_fail['encoding_correct'] = (first.encoding == cfg.get('thermal.encodings.image', '32FC1'))

        frames = [_load_frame(m) for _, m in msgs]
        finite_fractions = [float(np.mean(np.isfinite(f))) for f in frames]
        min_finite = cfg.required('thermal.thresholds.required.ut_thm_01_min_finite_fraction')
        metrics['min_finite_fraction'] = min(finite_fractions)
        pass_fail['finite_fraction_sufficient'] = min(finite_fractions) >= min_finite
        thresholds_used['finite_fraction_sufficient'] = {
            'value': min(finite_fractions), 'threshold': min_finite, 'tier': 'required'}

        stats = compute_rate_stats([_header_stamp_ns(m) for _, m in msgs])
        metrics['rate'] = stats.to_dict()
        min_rate = cfg.required('thermal.thresholds.required.ut_thm_01_min_rate_hz')
        max_rate = cfg.required('thermal.thresholds.required.ut_thm_01_max_rate_hz')
        pass_fail['rate_within_band'] = min_rate <= stats.mean_hz <= max_rate
        thresholds_used['rate_within_band'] = {
            'value': stats.mean_hz, 'threshold': f'{min_rate}-{max_rate}', 'tier': 'required'}

    overall = all(pass_fail.values())
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-THM-01', metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _roi_indices(roi, width, height):
    if roi is None:
        return None
    x0, y0, x1, y1 = roi
    ys, xs = np.mgrid[y0:y1, x0:x1]
    return (ys * width + xs).flatten()


def _run_contrast_profile(bag, cfg, result_dir, ref_target_c, ref_background_c,
                           target_roi, background_roi) -> dict:
    image_topic = cfg.get('thermal.topics.image', '/thermal/image')
    msgs = list(bag.messages(image_topic))
    if not msgs:
        write_metrics_json(
            result_dir.metrics_json_path, {'error': 'no thermal frames recorded'}, {'overall': False}
        )
        return {'pass': False}

    width = int(cfg.get('thermal.dimensions.width_px', 32))
    height = int(cfg.get('thermal.dimensions.height_px', 24))
    frames = np.stack([_load_frame(m).flatten() for _, m in msgs])  # (n_frames, w*h)

    target_idx = _roi_indices(target_roi, width, height)
    background_idx = _roi_indices(background_roi, width, height)
    target_frames = frames if target_idx is None else frames[:, target_idx]
    background_frames = frames if background_idx is None else frames[:, background_idx]

    bias = target_background_bias(target_frames, background_frames, ref_target_c, ref_background_c)
    cnr = contrast_to_noise_ratio(target_frames, background_frames)
    p_cnr = pooled_cnr(target_frames, background_frames)

    per_frame_target_mean = np.nanmean(target_frames, axis=1)
    drift_c = drift(per_frame_target_mean)
    temporal_noise_arr = temporal_noise(target_frames)

    has_nan_inf = bool(np.any(~np.isfinite(target_frames)) or np.any(~np.isfinite(background_frames)))

    metrics = {
        'target_bias_c': bias['target_bias_c'],
        'background_bias_c': bias['background_bias_c'],
        'cnr': cnr,
        'pooled_cnr': p_cnr,
        'drift_c': drift_c,
        'mean_temporal_noise_c': float(np.mean(temporal_noise_arr)),
        'has_nan_or_inf_in_roi': has_nan_inf,
    }

    max_target_bias = cfg.provisional('thermal.thresholds.provisional.ut_thm_02_max_target_bias_c', 2.0)
    max_bg_bias = cfg.provisional('thermal.thresholds.provisional.ut_thm_02_max_background_bias_c', 2.0)
    min_cnr = cfg.provisional('thermal.thresholds.provisional.ut_thm_02_min_cnr', 5.0)
    max_drift = cfg.required('thermal.thresholds.required.ut_thm_02_max_drift_c')

    pass_fail = {
        'target_bias_within_provisional_limit': abs(bias['target_bias_c']) <= max_target_bias,
        'background_bias_within_provisional_limit': abs(bias['background_bias_c']) <= max_bg_bias,
        'cnr_within_provisional_limit': cnr >= min_cnr,
        'no_nan_inf_in_roi': not has_nan_inf,
        'drift_within_limit': abs(drift_c) <= max_drift,
    }
    thresholds_used = {
        'target_bias_within_provisional_limit': {
            'value': bias['target_bias_c'], 'threshold': max_target_bias, 'tier': 'provisional'},
        'background_bias_within_provisional_limit': {
            'value': bias['background_bias_c'], 'threshold': max_bg_bias, 'tier': 'provisional'},
        'cnr_within_provisional_limit': {'value': cnr, 'threshold': min_cnr, 'tier': 'provisional'},
        'no_nan_inf_in_roi': {'value': has_nan_inf, 'threshold': False, 'tier': 'required'},
        'drift_within_limit': {'value': drift_c, 'threshold': max_drift, 'tier': 'required'},
    }
    # Bias/CNR are provisional per spec -- reported, but only NaN/Inf and drift (both
    # required) can fail the run by themselves.
    required_pass = pass_fail['no_nan_inf_in_roi'] and pass_fail['drift_within_limit']
    pass_fail['overall'] = required_pass

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-THM-02', metrics, thresholds_used, pass_fail)
    )
    return {'pass': required_pass}


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _parse_roi(value: str):
    if not value:
        return None
    x0, y0, x1, y1 = (int(v) for v in value.split(','))
    return (x0, y0, x1, y1)


def _self_test() -> int:
    rng = np.random.default_rng(0)
    target = 35.0 + rng.normal(0, 0.3, size=200)
    background = 22.0 + rng.normal(0, 0.3, size=200)
    cnr = contrast_to_noise_ratio(target, background)
    print(f'[self-test] synthetic target/background CNR = {cnr:.2f}')
    ok = cnr >= 5.0
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    parser.add_argument('--profile', choices=['stream', 'contrast'], default='stream')
    parser.add_argument('--ref-target-c', type=float, default=35.0)
    parser.add_argument('--ref-background-c', type=float, default=22.0)
    parser.add_argument('--target-roi', default='', help='x0,y0,x1,y1 pixel ROI')
    parser.add_argument('--background-roi', default='', help='x0,y0,x1,y1 pixel ROI')
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
            result_dir.metrics_json_path, {'error': 'bag is empty or missing'}, {'overall': False}
        )
        print('ERROR: bag is empty or missing', file=sys.stderr)
        return 1

    if args.profile == 'stream':
        result = _run_stream_profile(bag, cfg, result_dir)
    else:
        result = _run_contrast_profile(
            bag, cfg, result_dir, args.ref_target_c, args.ref_background_c,
            _parse_roi(args.target_roi), _parse_roi(args.background_roi),
        )

    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
