#!/usr/bin/env python3
"""CLI: UT-THM-01 stream check (--profile stream) and UT-THM-02 target/background
contrast (--profile contrast). Reads recorded rosbag2 data offline via BagReader."""

import argparse
import json
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


def _validate_roi(roi, width, height, name):
    """Validate an x0,y0,x1,y1 ROI using exclusive x1/y1 bounds."""
    if roi is None:
        raise ValueError(f'{name} ROI is required for --profile contrast')

    x0, y0, x1, y1 = roi

    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError(
            f'{name} ROI {roi} is outside the valid {width}x{height} image bounds '
            f'or has zero/negative area'
        )

    return roi


def _rois_overlap(a, b):
    """Return True if two rectangular ROIs overlap."""
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b

    return not (
        ax1 <= bx0 or
        bx1 <= ax0 or
        ay1 <= by0 or
        by1 <= ay0
    )


def _roi_indices(roi, width, height):
    x0, y0, x1, y1 = roi
    ys, xs = np.mgrid[y0:y1, x0:x1]
    return (ys * width + xs).flatten()


def _run_contrast_profile(
    bag,
    cfg,
    result_dir,
    ref_target_c,
    ref_background_c,
    target_roi,
    background_roi,
    ref_uncertainty_c,
) -> dict:
    """
    UT-THM-02 stationary target/background characterization.

    The camera, warm target, and background remain stationary for the full
    recording. The same target and background ROIs are therefore evaluated
    across every recorded thermal frame.
    """
    image_topic = cfg.get('thermal.topics.image', '/thermal/image')
    msgs = list(bag.messages(image_topic))

    if not msgs:
        write_metrics_json(
            result_dir.metrics_json_path,
            {'error': 'no thermal frames recorded'},
            {'overall': False},
        )
        return {'pass': False}

    width = int(cfg.get('thermal.dimensions.width_px', 32))
    height = int(cfg.get('thermal.dimensions.height_px', 24))

    # Require explicit, valid, non-overlapping ROIs.
    try:
        target_roi = _validate_roi(target_roi, width, height, 'target')
        background_roi = _validate_roi(background_roi, width, height, 'background')

        if _rois_overlap(target_roi, background_roi):
            raise ValueError(
                f'target ROI {target_roi} overlaps background ROI {background_roi}'
            )

    except ValueError as e:
        error = str(e)
        print(f'ERROR: {error}', file=sys.stderr)
        write_metrics_json(
            result_dir.metrics_json_path,
            {'error': error},
            {'overall': False},
        )
        return {'pass': False}

    # Load the complete stationary thermal sequence.
    frames = np.stack(
        [_load_frame(m).flatten() for _, m in msgs]
    )  # shape: (n_frames, 768)

    target_idx = _roi_indices(target_roi, width, height)
    background_idx = _roi_indices(background_roi, width, height)

    target_frames = frames[:, target_idx]
    background_frames = frames[:, background_idx]

    # Check raw-data integrity before calculating statistics.
    has_nan_inf = bool(
        np.any(~np.isfinite(target_frames)) or
        np.any(~np.isfinite(background_frames))
    )

    # Bias against independently measured reference temperatures.
    bias = target_background_bias(
        target_frames,
        background_frames,
        ref_target_c,
        ref_background_c,
    )

    # Aggregate contrast metrics.
    cnr = contrast_to_noise_ratio(target_frames, background_frames)
    p_cnr = pooled_cnr(target_frames, background_frames)

    target_mean_c = float(np.nanmean(target_frames))
    target_median_c = float(np.nanmedian(target_frames))
    background_mean_c = float(np.nanmean(background_frames))
    background_median_c = float(np.nanmedian(background_frames))
    delta_t_c = target_mean_c - background_mean_c

    # Spatial nonuniformity within each ROI, calculated per frame and then
    # averaged over the full stationary capture.
    per_frame_target_spatial_std = np.nanstd(target_frames, axis=1)
    per_frame_background_spatial_std = np.nanstd(background_frames, axis=1)

    mean_target_spatial_std_c = float(
        np.nanmean(per_frame_target_spatial_std)
    )
    mean_background_spatial_std_c = float(
        np.nanmean(per_frame_background_spatial_std)
    )

    # Temporal mean histories.
    per_frame_target_mean = np.nanmean(target_frames, axis=1)
    per_frame_background_mean = np.nanmean(background_frames, axis=1)

    # Drift over the stationary sequence.
    target_drift_c = drift(per_frame_target_mean)
    background_drift_c = drift(per_frame_background_mean)

    # Per-pixel temporal noise over the stationary sequence.
    target_temporal_noise = temporal_noise(target_frames)
    background_temporal_noise = temporal_noise(background_frames)

    target_temporal_noise_mean_c = float(
        np.nanmean(target_temporal_noise)
    )
    target_temporal_noise_median_c = float(
        np.nanmedian(target_temporal_noise)
    )
    background_temporal_noise_mean_c = float(
        np.nanmean(background_temporal_noise)
    )
    background_temporal_noise_median_c = float(
        np.nanmedian(background_temporal_noise)
    )

    # Capture timing information for traceability.
    stamps_ns = [_header_stamp_ns(m) for _, m in msgs]

    if len(stamps_ns) > 1:
        duration_s = (stamps_ns[-1] - stamps_ns[0]) / 1e9
        observed_rate_hz = (len(stamps_ns) - 1) / duration_s
    else:
        duration_s = 0.0
        observed_rate_hz = 0.0

    # The test plan says bias is characterization-only if the reference
    # uncertainty is greater than 1 C.
    characterization_only = ref_uncertainty_c > 1.0

    metrics = {
        'frame_count': len(msgs),
        'duration_s': duration_s,
        'observed_rate_hz': observed_rate_hz,

        'target_roi': list(target_roi),
        'background_roi': list(background_roi),
        'target_roi_pixel_count': int(len(target_idx)),
        'background_roi_pixel_count': int(len(background_idx)),

        'reference_target_c': ref_target_c,
        'reference_background_c': ref_background_c,
        'reference_uncertainty_c': ref_uncertainty_c,
        'characterization_only_due_to_reference_uncertainty': characterization_only,

        'target_mean_c': target_mean_c,
        'target_median_c': target_median_c,
        'background_mean_c': background_mean_c,
        'background_median_c': background_median_c,
        'delta_t_c': delta_t_c,

        'target_bias_c': bias['target_bias_c'],
        'background_bias_c': bias['background_bias_c'],

        'cnr': cnr,
        'pooled_cnr': p_cnr,

        'mean_target_spatial_std_c': mean_target_spatial_std_c,
        'mean_background_spatial_std_c': mean_background_spatial_std_c,

        'target_temporal_noise_mean_c': target_temporal_noise_mean_c,
        'target_temporal_noise_median_c': target_temporal_noise_median_c,
        'background_temporal_noise_mean_c': background_temporal_noise_mean_c,
        'background_temporal_noise_median_c': background_temporal_noise_median_c,

        'target_drift_c': target_drift_c,
        'background_drift_c': background_drift_c,

        'has_nan_or_inf_in_roi': has_nan_inf,
    }

    # Persist exactly what was supplied to the analysis so the result is
    # reproducible without reconstructing the CLI command later.
    analysis_inputs = {
        'profile': 'contrast',
        'method': 'stationary_target_background_full_capture',
        'target_roi': list(target_roi),
        'background_roi': list(background_roi),
        'roi_coordinate_convention': 'x0,y0,x1,y1; x1/y1 exclusive',
        'ref_target_c': ref_target_c,
        'ref_background_c': ref_background_c,
        'ref_uncertainty_c': ref_uncertainty_c,
    }

    analysis_inputs_path = (
        result_dir.exports_dir / 'ut_thm_02_analysis_inputs.json'
    )

    analysis_inputs_path.write_text(
        json.dumps(analysis_inputs, indent=2) + '\n'
    )

    # Acceptance limits.
    max_target_bias = cfg.provisional(
        'thermal.thresholds.provisional.ut_thm_02_max_target_bias_c',
        2.0,
    )
    max_bg_bias = cfg.provisional(
        'thermal.thresholds.provisional.ut_thm_02_max_background_bias_c',
        2.0,
    )
    min_cnr = cfg.provisional(
        'thermal.thresholds.provisional.ut_thm_02_min_cnr',
        5.0,
    )
    max_drift = cfg.required(
        'thermal.thresholds.required.ut_thm_02_max_drift_c'
    )

    target_bias_ok = abs(bias['target_bias_c']) <= max_target_bias
    background_bias_ok = abs(bias['background_bias_c']) <= max_bg_bias
    cnr_ok = cnr >= min_cnr
    target_drift_ok = abs(target_drift_c) <= max_drift
    background_drift_ok = abs(background_drift_c) <= max_drift
    finite_ok = not has_nan_inf

    pass_fail = {
        'target_bias_within_provisional_limit': target_bias_ok,
        'background_bias_within_provisional_limit': background_bias_ok,
        'cnr_within_provisional_limit': cnr_ok,
        'no_nan_inf_in_roi': finite_ok,
        'target_drift_within_limit': target_drift_ok,
        'background_drift_within_limit': background_drift_ok,
    }

    thresholds_used = {
        'target_bias_within_provisional_limit': {
            'value': bias['target_bias_c'],
            'threshold': max_target_bias,
            'tier': 'provisional',
        },
        'background_bias_within_provisional_limit': {
            'value': bias['background_bias_c'],
            'threshold': max_bg_bias,
            'tier': 'provisional',
        },
        'cnr_within_provisional_limit': {
            'value': cnr,
            'threshold': min_cnr,
            'tier': 'provisional',
        },
        'no_nan_inf_in_roi': {
            'value': has_nan_inf,
            'threshold': False,
            'tier': 'required',
        },
        'target_drift_within_limit': {
            'value': target_drift_c,
            'threshold': max_drift,
            'tier': 'required',
        },
        'background_drift_within_limit': {
            'value': background_drift_c,
            'threshold': max_drift,
            'tier': 'required',
        },
    }

    # For the formal initial UT-THM-02 bench acceptance, CNR and the
    # provisional bias gates are included in the overall verdict.
    #
    # If reference uncertainty exceeds 1 C, the two bias gates are reported
    # but do not fail the sensor; the result is explicitly marked as
    # characterization-only for absolute temperature accuracy.
    required_pass = (
        finite_ok and
        target_drift_ok and
        background_drift_ok
    )

    contrast_pass = cnr_ok

    if characterization_only:
        bias_pass = True
    else:
        bias_pass = target_bias_ok and background_bias_ok

    overall = required_pass and contrast_pass and bias_pass
    pass_fail['overall'] = overall

    write_metrics_json(
        result_dir.metrics_json_path,
        metrics,
        pass_fail,
    )
    write_metrics_csv(
        result_dir.metrics_csv_path,
        metrics,
    )
    result_dir.report_path.write_text(
        render_report_md(
            'UT-THM-02',
            metrics,
            thresholds_used,
            pass_fail,
        )
    )

    return {'pass': overall}


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
    parser.add_argument(
    '--ref-target-c',
    type=float,
    default=None,
    help='measured target reference temperature in degrees C',
    )
    parser.add_argument(
        '--ref-background-c',
        type=float,
        default=None,
        help='measured background reference temperature in degrees C',
    )
    parser.add_argument(
        '--ref-uncertainty-c',
        type=float,
        default=None,
        help='estimated +/- uncertainty of the reference temperature measurement in degrees C',
    )
    parser.add_argument(
        '--target-roi',
        default='',
        help='target ROI as x0,y0,x1,y1; x1/y1 are exclusive',
    )
    parser.add_argument(
        '--background-roi',
        default='',
        help='background ROI as x0,y0,x1,y1; x1/y1 are exclusive',
    )
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
        if args.ref_target_c is None:
            parser.error('--ref-target-c is required for --profile contrast')

        if args.ref_background_c is None:
            parser.error('--ref-background-c is required for --profile contrast')

        if args.ref_uncertainty_c is None:
            parser.error('--ref-uncertainty-c is required for --profile contrast')

        if not args.target_roi:
            parser.error('--target-roi is required for --profile contrast')

        if not args.background_roi:
            parser.error('--background-roi is required for --profile contrast')

        result = _run_contrast_profile(
            bag,
            cfg,
            result_dir,
            args.ref_target_c,
            args.ref_background_c,
            _parse_roi(args.target_roi),
            _parse_roi(args.background_roi),
            args.ref_uncertainty_c,
        )

    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
