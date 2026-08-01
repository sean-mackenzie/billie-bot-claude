#!/usr/bin/env python3
"""CLI: DT-OAK-01 scoring against the production oakd_dog_detector. Reads recorded
rosbag2 data + operator ground-truth segments (exports/ground_truth_segments.csv)
offline via BagReader -- never requires a live ROS node."""

import argparse
import csv
import os
import sys

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.ground_truth_marker import parse_quantity
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats
from billiebot_sensor_tests.common.report import render_report_md, write_metrics_csv, write_metrics_json
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.oakd.detection_scoring import (
    DetectionSample,
    GroundTruthSegment,
    compute_detection_metrics,
)


def _load_ground_truth_segments(result_dir) -> list:
    """Reads exports/ground_truth_segments.csv (written by ground_truth_marker_node) and
    turns consecutive 'mark' rows into [t_start, t_end) segments -- each row's t_start_ns
    is the start of that labeled condition, ending at the next row's t_start_ns (the last
    row is given a 30s default duration)."""
    path = result_dir.exports_dir / 'ground_truth_segments.csv'
    if not path.exists():
        return []
    with open(path, 'r') as f:
        rows = list(csv.DictReader(f))
    segments = []
    for i, row in enumerate(rows):
        t_start = int(row['t_start_ns'])
        t_end = int(rows[i + 1]['t_start_ns']) if i + 1 < len(rows) else t_start + int(30e9)
        dog_present = row['label'].strip().lower() not in ('', 'empty', 'negative', 'no_dog')
        distance_m = None
        if row.get('distance_m'):
            try:
                distance_m = parse_quantity(row['distance_m'])
            except ValueError:
                distance_m = None
        segments.append(GroundTruthSegment(t_start, t_end, dog_present, distance_m))
    return segments


def _load_detections(bag, topic) -> list:
    samples = []
    for t_ns, msg in bag.messages(topic):
        samples.append(DetectionSample(
            t_ns=t_ns,
            bbox=(msg.bbox_x, msg.bbox_y, msg.bbox_w, msg.bbox_h),
            confidence=msg.confidence,
            depth_m=msg.depth,
        ))
    return samples


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    detections = [
        DetectionSample(t_ns=i * int(0.2e9), bbox=(10, 10, 50, 50), confidence=0.9, depth_m=2.0)
        for i in range(10)
    ]
    segments = [GroundTruthSegment(0, int(2.0e9), True, 2.0)]
    metrics = compute_detection_metrics(detections, segments)
    print(f"[self-test] recall = {metrics['recall']:.2f}")
    ok = metrics['recall'] == 1.0
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
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

    detections_topic = cfg.get('oakd.topics.detections', '/dog/detections_3d')
    found_topic = cfg.get('oakd.topics.found', '/dog/found')

    detections = _load_detections(bag, detections_topic)
    segments = _load_ground_truth_segments(result_dir)

    min_confidence = cfg.get('oakd.confidence_threshold', 0.5)
    reliable_fraction = cfg.required(
        'oakd.thresholds.required.dt_oak_01_reliable_detection_fraction'
    )
    detection_metrics = compute_detection_metrics(
        detections, segments, min_confidence=min_confidence, reliable_fraction=reliable_fraction
    )

    found_msgs = list(bag.messages(found_topic))
    found_rate_stats = compute_rate_stats([t for t, _ in found_msgs]) if found_msgs else None

    metrics = {
        'detection': detection_metrics,
        'found_rate': found_rate_stats.to_dict() if found_rate_stats else None,
        'n_segments': len(segments),
        'n_detections': len(detections),
    }

    min_found_hz = cfg.required('oakd.thresholds.required.dt_oak_01_min_found_rate_hz')
    max_found_hz = cfg.required('oakd.thresholds.required.dt_oak_01_max_found_rate_hz')
    min_recall = cfg.required('oakd.thresholds.required.dt_oak_01_min_recall')
    max_depth_error = cfg.required('oakd.thresholds.required.dt_oak_01_max_depth_error_at_2m_m')
    max_fp_provisional = cfg.provisional(
        'oakd.thresholds.provisional.dt_oak_01_max_empty_scene_false_positive_fraction', 0.05
    )

    pass_fail = {}
    thresholds_used = {}

    if found_rate_stats:
        pass_fail['found_rate_within_band'] = (
            min_found_hz <= found_rate_stats.mean_hz <= max_found_hz
        )
        thresholds_used['found_rate_within_band'] = {
            'value': found_rate_stats.mean_hz,
            'threshold': f'{min_found_hz}-{max_found_hz}', 'tier': 'required'}
    else:
        pass_fail['found_rate_within_band'] = False
        thresholds_used['found_rate_within_band'] = {
            'value': None, 'threshold': f'{min_found_hz}-{max_found_hz}', 'tier': 'required'}

    recall = detection_metrics['recall']
    pass_fail['recall_sufficient'] = (recall == recall) and recall >= min_recall  # NaN-safe
    thresholds_used['recall_sufficient'] = {
        'value': recall, 'threshold': min_recall, 'tier': 'required'}

    depth_err = detection_metrics['median_depth_error_m']
    pass_fail['depth_error_within_limit'] = (depth_err == depth_err) and depth_err <= max_depth_error
    thresholds_used['depth_error_within_limit'] = {
        'value': depth_err, 'threshold': max_depth_error, 'tier': 'required'}

    valid_bbox_frac = detection_metrics['valid_bbox_fraction']
    pass_fail['valid_bbox_fraction_is_one'] = (
        valid_bbox_frac != valid_bbox_frac or valid_bbox_frac == 1.0  # NaN = vacuously true
    )
    thresholds_used['valid_bbox_fraction_is_one'] = {
        'value': valid_bbox_frac, 'threshold': 1.0, 'tier': 'required'}

    fp_frac = detection_metrics['false_positive_fraction']
    pass_fail['empty_scene_fp_within_provisional_limit'] = fp_frac <= max_fp_provisional
    thresholds_used['empty_scene_fp_within_provisional_limit'] = {
        'value': fp_frac, 'threshold': max_fp_provisional, 'tier': 'provisional'}

    required_checks = [v for k, v in pass_fail.items() if thresholds_used[k]['tier'] == 'required']
    overall = all(required_checks)
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('DT-OAK-01', metrics, thresholds_used, pass_fail)
    )

    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
