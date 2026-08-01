#!/usr/bin/env python3
"""CLI: DT-AUD-01 classification scoring (--profile classification) and DT-AUD-02 DoA
scoring (--profile doa) against the production audio_classifier. Reads recorded rosbag2
data + operator ground-truth segments offline via BagReader.

DT-AUD-01 and DT-AUD-02 are independent data-collection sessions (different ground-truth
marks: bark/speech/noise trials vs. bearing sweeps) -- run audio_classifier_bench.launch.py
twice, once per test, each with its own --results-dir, then score each with the matching
--profile. This keeps each test's metrics.json/report.md independent, so a DT-AUD-02
failure can never affect DT-AUD-01's result (per spec: "classifier pass does not depend on
this optional test").
"""

import argparse
import csv
import os
import sys

import numpy as np

from billiebot_sensor_tests.audio.classifier_scoring import (
    label_distribution,
    latency_stats,
    precision_recall_f1,
)
from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.circular_stats import circular_abs_error_deg
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.ground_truth_marker import parse_quantity
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats
from billiebot_sensor_tests.common.report import render_report_md, write_metrics_csv, write_metrics_json
from billiebot_sensor_tests.common.result_dir import BenchResultDir

_EVENT_TYPE_NAMES = {0: 'BARK', 1: 'WHINE', 2: 'HOWL', 3: 'LOUD_NOISE', 4: 'SILENCE'}
_TRUE_LABEL_MAP = {
    'bark': 'BARK', 'billie_bark': 'BARK', 'whine': 'WHINE', 'howl': 'HOWL',
    'speech': 'SPEECH', 'noise': 'LOUD_NOISE', 'impulse': 'LOUD_NOISE',
    'silence': 'SILENCE', 'ambient': 'SILENCE',
}


def _normalize_true_label(raw_label: str) -> str:
    return _TRUE_LABEL_MAP.get(raw_label.strip().lower(), raw_label.strip().upper() or 'SILENCE')


def _load_ground_truth_segments(result_dir) -> list:
    path = result_dir.exports_dir / 'ground_truth_segments.csv'
    if not path.exists():
        return []
    with open(path, 'r') as f:
        rows = list(csv.DictReader(f))
    segments = []
    for i, row in enumerate(rows):
        t_start = int(row['t_start_ns'])
        t_end = int(rows[i + 1]['t_start_ns']) if i + 1 < len(rows) else t_start + int(15e9)
        segments.append({
            't_start_ns': t_start, 't_end_ns': t_end, 'label': row['label'].strip(),
            'distance_m': row.get('distance_m', ''), 'orientation': row.get('orientation', ''),
        })
    return segments


def _status_kv(msg) -> dict:
    return {v.key: v.value for v in msg.status[0].values} if msg.status else {}


def _run_classification_profile(bag, cfg, result_dir) -> dict:
    events_topic = cfg.get('audio.topics.events', '/audio/events')
    status_topic = cfg.get('audio.topics.status', '/bench/audio_classifier/status')

    status_msgs = list(bag.messages(status_topic))
    event_msgs = list(bag.messages(events_topic))
    segments = _load_ground_truth_segments(result_dir)

    metrics = {'n_status_cycles': len(status_msgs), 'n_events': len(event_msgs)}
    pass_fail = {}
    thresholds_used = {}

    min_status_hz = cfg.required('audio.thresholds.required.dt_aud_01_min_status_rate_hz')
    max_status_hz = cfg.required('audio.thresholds.required.dt_aud_01_max_status_rate_hz')
    if status_msgs:
        status_stats = compute_rate_stats([t for t, _ in status_msgs])
        metrics['status_rate'] = status_stats.to_dict()
        overrun_values, inference_durations = [], []
        for _, msg in status_msgs:
            kv = _status_kv(msg)
            if 'buffer_overrun_count' in kv:
                overrun_values.append(int(kv['buffer_overrun_count']))
            if 'inference_duration_s' in kv:
                inference_durations.append(float(kv['inference_duration_s']))
        max_overrun = max(overrun_values) if overrun_values else 0
        metrics['max_buffer_overrun_count'] = max_overrun
        metrics['inference_duration'] = latency_stats(inference_durations)
        pass_fail['status_rate_within_band'] = min_status_hz <= status_stats.mean_hz <= max_status_hz
        pass_fail['no_buffer_overruns'] = max_overrun == 0
        thresholds_used['status_rate_within_band'] = {
            'value': status_stats.mean_hz,
            'threshold': f'{min_status_hz}-{max_status_hz}', 'tier': 'required'}
        thresholds_used['no_buffer_overruns'] = {
            'value': max_overrun, 'threshold': 0, 'tier': 'required'}
    else:
        pass_fail['status_rate_within_band'] = False
        pass_fail['no_buffer_overruns'] = False
        thresholds_used['status_rate_within_band'] = {
            'value': None, 'threshold': f'{min_status_hz}-{max_status_hz}', 'tier': 'required'}
        thresholds_used['no_buffer_overruns'] = {'value': None, 'threshold': 0, 'tier': 'required'}

    predicted_labels, true_labels, latencies = [], [], []
    for seg in segments:
        true_label = _normalize_true_label(seg['label'])
        seg_events = [(t, m) for t, m in event_msgs if seg['t_start_ns'] <= t < seg['t_end_ns']]
        if seg_events:
            t_ns, m = seg_events[0]
            predicted_labels.append(_EVENT_TYPE_NAMES.get(m.event_type, 'LOUD_NOISE'))
            true_labels.append(true_label)
            latencies.append(max((t_ns - seg['t_end_ns']) / 1e9, 0.0))
        elif true_label != 'SILENCE':
            predicted_labels.append('NO_EVENT')
            true_labels.append(true_label)

    if predicted_labels:
        bark_metrics = precision_recall_f1(predicted_labels, true_labels, positive_label='BARK')
        metrics['bark_precision_recall_f1'] = bark_metrics
        metrics['event_latency'] = latency_stats(latencies)
        metrics['yamnet_label_distribution_all_events'] = (
            label_distribution([m.yamnet_label for _, m in event_msgs]) if event_msgs else {}
        )

        min_recall = cfg.required('audio.thresholds.required.dt_aud_01_min_bark_recall')
        recall = bark_metrics['recall']
        pass_fail['bark_recall_sufficient'] = (recall == recall) and recall >= min_recall
        thresholds_used['bark_recall_sufficient'] = {
            'value': recall, 'threshold': min_recall, 'tier': 'required'}

        non_bark_true = [(p, t) for p, t in zip(predicted_labels, true_labels) if t != 'BARK']
        false_bark = sum(1 for p, t in non_bark_true if p == 'BARK')
        false_bark_rate = (false_bark / len(non_bark_true)) if non_bark_true else 0.0
        max_false_bark = cfg.provisional(
            'audio.thresholds.provisional.dt_aud_01_max_false_bark_rate', 0.10
        )
        metrics['false_bark_rate'] = false_bark_rate
        pass_fail['false_bark_rate_within_provisional_limit'] = false_bark_rate <= max_false_bark
        thresholds_used['false_bark_rate_within_provisional_limit'] = {
            'value': false_bark_rate, 'threshold': max_false_bark, 'tier': 'provisional'}

        max_latency = cfg.provisional(
            'audio.thresholds.provisional.dt_aud_01_max_event_latency_s', 1.5
        )
        worst_latency = max(latencies) if latencies else 0.0
        pass_fail['event_latency_within_provisional_limit'] = worst_latency <= max_latency
        thresholds_used['event_latency_within_provisional_limit'] = {
            'value': worst_latency, 'threshold': max_latency, 'tier': 'provisional'}

    silence_segments = [s for s in segments if _normalize_true_label(s['label']) == 'SILENCE']
    silence_has_no_events = all(
        not any(seg['t_start_ns'] <= t < seg['t_end_ns'] for t, _ in event_msgs)
        for seg in silence_segments
    ) if silence_segments else True
    pass_fail['silence_produces_no_events'] = silence_has_no_events
    thresholds_used['silence_produces_no_events'] = {
        'value': silence_has_no_events, 'threshold': True, 'tier': 'required'}

    required_checks = [v for k, v in pass_fail.items() if thresholds_used[k]['tier'] == 'required']
    overall = all(required_checks) if required_checks else False
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('DT-AUD-01', metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _run_doa_profile(bag, cfg, result_dir) -> dict:
    events_topic = cfg.get('audio.topics.events', '/audio/events')
    event_msgs = list(bag.messages(events_topic))
    segments = _load_ground_truth_segments(result_dir)

    pairs = []  # (measured_doa_deg, true_bearing_deg)
    for seg in segments:
        orientation = seg.get('orientation', '')
        if not orientation:
            continue
        try:
            true_bearing = parse_quantity(orientation)
        except ValueError:
            continue
        for t_ns, m in event_msgs:
            if seg['t_start_ns'] <= t_ns < seg['t_end_ns']:
                pairs.append((m.doa_deg, true_bearing))

    metrics = {'n_samples': len(pairs)}
    if pairs:
        measured = np.array([p[0] for p in pairs])
        true = np.array([p[1] for p in pairs])
        errors = circular_abs_error_deg(measured, true)
        metrics['mean_abs_circular_error_deg'] = float(np.mean(errors))
        metrics['within_15deg_fraction'] = float(np.mean(errors <= 15.0))
        metrics['doa_all_zero'] = bool(np.all(measured == 0.0))
    else:
        metrics['mean_abs_circular_error_deg'] = float('nan')
        metrics['within_15deg_fraction'] = float('nan')
        metrics['doa_all_zero'] = True

    max_error = cfg.provisional(
        'audio.thresholds.provisional.dt_aud_02_max_mean_circular_error_deg', 15.0
    )
    min_within = cfg.provisional(
        'audio.thresholds.provisional.dt_aud_02_min_within_15deg_fraction', 0.80
    )

    mean_err = metrics['mean_abs_circular_error_deg']
    within_frac = metrics['within_15deg_fraction']
    pass_fail = {
        'mean_error_within_provisional_limit': (mean_err == mean_err) and mean_err <= max_error,
        'within_15deg_fraction_within_provisional_limit': (
            within_frac == within_frac and within_frac >= min_within
        ),
        'doa_not_fixed_at_zero': not metrics['doa_all_zero'],
    }
    thresholds_used = {
        'mean_error_within_provisional_limit': {
            'value': mean_err, 'threshold': max_error, 'tier': 'provisional'},
        'within_15deg_fraction_within_provisional_limit': {
            'value': within_frac, 'threshold': min_within, 'tier': 'provisional'},
        'doa_not_fixed_at_zero': {
            'value': metrics['doa_all_zero'], 'threshold': False, 'tier': 'required'},
    }
    required_checks = [v for k, v in pass_fail.items() if thresholds_used[k]['tier'] == 'required']
    overall = all(required_checks)
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('DT-AUD-02', metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    predicted = ['BARK'] * 8 + ['LOUD_NOISE'] * 2
    true = ['BARK'] * 10
    metrics = precision_recall_f1(predicted, true, positive_label='BARK')
    print(f"[self-test] bark recall = {metrics['recall']:.2f}")
    ok = metrics['recall'] == 0.8
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    parser.add_argument('--profile', choices=['classification', 'doa'], default='classification')
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

    if args.profile == 'classification':
        result = _run_classification_profile(bag, cfg, result_dir)
    else:
        result = _run_doa_profile(bag, cfg, result_dir)

    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
