#!/usr/bin/env python3
"""CLI: DT-THM-01 scoring against the production thermal_node's warm-body blob output
(unmodified -- plan section 5b). Reads recorded rosbag2 data + operator ground-truth
segments offline via BagReader."""

import argparse
import csv
import os
import sys

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.ground_truth_marker import parse_quantity
from billiebot_sensor_tests.common.report import render_report_md, write_metrics_csv, write_metrics_json
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.thermal.blob_scoring import (
    BlobSample,
    ThermalGroundTruthSegment,
    compute_blob_metrics,
)


def _load_ground_truth_segments(result_dir) -> list:
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
        segments.append(ThermalGroundTruthSegment(t_start, t_end, dog_present, distance_m))
    return segments


def _load_blobs(bag, topic) -> list:
    blobs = []
    for t_ns, msg in bag.messages(topic):
        blobs.append(BlobSample(
            t_ns=t_ns, cx=msg.cx, cy=msg.cy, area=msg.area,
            max_temp=msg.max_temp, mean_temp=msg.mean_temp,
            is_dog_candidate=msg.is_dog_candidate,
        ))
    return blobs


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    blobs = [
        BlobSample(t_ns=i * int(0.25e9), cx=16, cy=12, area=20, max_temp=36, mean_temp=35,
                   is_dog_candidate=True)
        for i in range(4)
    ]
    frame_ts = [i * int(0.25e9) for i in range(4)]
    segments = [ThermalGroundTruthSegment(0, int(1.0e9), True, 1.0)]
    metrics = compute_blob_metrics(blobs, frame_ts, segments)
    frac = metrics['per_segment_detection_fraction'][0]
    print(f'[self-test] detection fraction = {frac:.2f}')
    ok = frac == 1.0
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

    image_topic = cfg.get('thermal.topics.image', '/thermal/image')
    blob_topic = cfg.get('thermal.topics.blob', '/thermal/blob')

    frame_timestamps = [t_ns for t_ns, _ in bag.messages(image_topic)]
    blobs = _load_blobs(bag, blob_topic)
    segments = _load_ground_truth_segments(result_dir)

    area_min = int(cfg.required('thermal.thresholds.required.dt_thm_01_area_min_px'))
    temp_min = cfg.required('thermal.thresholds.required.dt_thm_01_temp_min_c')
    temp_max = cfg.required('thermal.thresholds.required.dt_thm_01_temp_max_c')
    reliable_fraction = cfg.provisional(
        'thermal.thresholds.provisional.dt_thm_01_min_detection_fraction', 0.80
    )

    blob_metrics = compute_blob_metrics(
        blobs, frame_timestamps, segments, area_min=area_min, temp_min_c=temp_min,
        temp_max_c=temp_max, reliable_fraction=reliable_fraction,
    )

    metrics = {
        'blob': blob_metrics,
        'n_segments': len(segments),
        'n_blobs': len(blobs),
        'n_frames': len(frame_timestamps),
    }

    negative_segments = [i for i, s in enumerate(segments) if not s.dog_present]
    empty_baseline_ok = all(
        blob_metrics['per_segment_detection_fraction'].get(i, 0.0) == 0.0
        for i in negative_segments
    ) if negative_segments else True

    positive_segments = [i for i, s in enumerate(segments) if s.dog_present]
    detection_fraction_ok = all(
        blob_metrics['per_segment_detection_fraction'].get(i, 0.0) >= reliable_fraction
        for i in positive_segments
    ) if positive_segments else False

    valid_output_frac = blob_metrics['valid_output_fraction']
    valid_output_ok = (valid_output_frac != valid_output_frac) or valid_output_frac == 1.0

    pass_fail = {
        'no_blobs_in_empty_baseline': empty_baseline_ok,
        'positive_detection_fraction_within_provisional_limit': detection_fraction_ok,
        'positive_outputs_valid': valid_output_ok,
    }
    thresholds_used = {
        'no_blobs_in_empty_baseline': {
            'value': empty_baseline_ok, 'threshold': True, 'tier': 'required'},
        'positive_detection_fraction_within_provisional_limit': {
            'value': detection_fraction_ok, 'threshold': reliable_fraction, 'tier': 'provisional'},
        'positive_outputs_valid': {
            'value': valid_output_ok, 'threshold': True, 'tier': 'required'},
    }
    required_checks = [v for k, v in pass_fail.items() if thresholds_used[k]['tier'] == 'required']
    overall = all(required_checks)
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('DT-THM-01', metrics, thresholds_used, pass_fail)
    )

    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
