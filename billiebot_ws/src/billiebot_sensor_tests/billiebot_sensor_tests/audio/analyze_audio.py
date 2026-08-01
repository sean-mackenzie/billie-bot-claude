#!/usr/bin/env python3
"""CLI: UT-AUD-01 WAV signal-quality analysis. Loads WAV file(s) directly (Python's wave
module + numpy) -- the audio evidence for this test is the exported WAV written by
audio_capture_node.py, not a rosbag2 topic."""

import argparse
import os
import sys
import wave
from pathlib import Path

import numpy as np

from billiebot_sensor_tests.audio.metrics import (
    channel_correlation,
    clipping_fraction,
    dc_offset,
    mains_hum_energy,
    peak_dbfs,
    rms_dbfs,
)
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.report import render_report_md, write_metrics_csv, write_metrics_json
from billiebot_sensor_tests.common.result_dir import BenchResultDir


def _load_wav(path) -> tuple:
    with wave.open(str(path), 'rb') as wf:
        sample_rate = wf.getframerate()
        n_channels = wf.getnchannels()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)
    pcm16 = np.frombuffer(raw, dtype=np.int16).reshape(-1, n_channels) if n_frames else \
        np.zeros((0, n_channels), dtype=np.int16)
    samples = pcm16.astype(np.float64) / 32768.0
    return samples, sample_rate, n_channels


def _analyze_one_wav(path) -> dict:
    samples, sample_rate, n_channels = _load_wav(path)
    duration_s = samples.shape[0] / sample_rate if sample_rate else 0.0

    per_channel = []
    for c in range(n_channels):
        ch = samples[:, c]
        per_channel.append({
            'channel': c,
            'rms_dbfs': rms_dbfs(ch),
            'peak_dbfs': peak_dbfs(ch),
            'clipping_fraction': clipping_fraction(ch),
            'dc_offset': dc_offset(ch),
            'mains_hum_energy': mains_hum_energy(ch, sample_rate),
        })

    channel_corr = (
        channel_correlation(samples[:, 0], samples[:, 1]) if n_channels >= 2 else None
    )

    return {
        'file': str(path),
        'sample_rate_hz': sample_rate,
        'channel_count': n_channels,
        'sample_count': int(samples.shape[0]),
        'duration_s': duration_s,
        'per_channel': per_channel,
        'channel_correlation': channel_corr,
    }


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    sample_rate = 16000
    t = np.arange(sample_rate) / sample_rate
    tone = 0.5 * np.sin(2 * np.pi * 440.0 * t)
    level = rms_dbfs(tone)
    print(f'[self-test] synthetic 440Hz tone RMS = {level:.2f} dBFS')
    ok = -20 < level < 0
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    parser.add_argument('--wav', action='append', default=[],
                         help='One or more WAV files to analyze (default: '
                              '<results_dir>/exports/*.wav)')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args(argv)

    if args.self_test:
        return _self_test()

    if not args.results_dir:
        parser.error('--results-dir is required unless --self-test is given')

    result_dir = BenchResultDir(args.results_dir)
    cfg = load_bench_config(args.config_file or _default_config_path())

    wav_paths = (
        [Path(p) for p in args.wav] if args.wav else sorted(result_dir.exports_dir.glob('*.wav'))
    )
    if not wav_paths:
        write_metrics_json(
            result_dir.metrics_json_path, {'error': 'no WAV files found'}, {'overall': False}
        )
        print('ERROR: no WAV files found', file=sys.stderr)
        return 1

    per_file = {p.stem: _analyze_one_wav(p) for p in wav_paths}

    expected_rate = cfg.required('audio.thresholds.required.ut_aud_01_sample_rate_hz')
    duration_tol = cfg.required('audio.thresholds.required.ut_aud_01_duration_tolerance_s')
    expected_duration = cfg.get('audio.duration_s', 3.0)
    max_clip = cfg.required('audio.thresholds.required.ut_aud_01_max_clipping_fraction')
    max_dc = cfg.provisional('audio.thresholds.provisional.ut_aud_01_max_abs_dc_offset', 0.01)

    pass_fail = {}
    thresholds_used = {}
    for stem, result in per_file.items():
        pass_fail[f'{stem}_sample_rate'] = result['sample_rate_hz'] == expected_rate
        pass_fail[f'{stem}_duration'] = abs(result['duration_s'] - expected_duration) <= duration_tol
        worst_clip = max((c['clipping_fraction'] for c in result['per_channel']), default=0.0)
        pass_fail[f'{stem}_clipping'] = worst_clip <= max_clip
        worst_dc = max((abs(c['dc_offset']) for c in result['per_channel']), default=0.0)
        pass_fail[f'{stem}_dc_offset_provisional'] = worst_dc <= max_dc
        thresholds_used[f'{stem}_sample_rate'] = {
            'value': result['sample_rate_hz'], 'threshold': expected_rate, 'tier': 'required'}
        thresholds_used[f'{stem}_duration'] = {
            'value': result['duration_s'], 'threshold': expected_duration, 'tier': 'required'}
        thresholds_used[f'{stem}_clipping'] = {
            'value': worst_clip, 'threshold': max_clip, 'tier': 'required'}
        thresholds_used[f'{stem}_dc_offset_provisional'] = {
            'value': worst_dc, 'threshold': max_dc, 'tier': 'provisional'}

    required_checks = [
        v for k, v in pass_fail.items()
        if thresholds_used.get(k, {}).get('tier') == 'required'
    ]
    overall = all(required_checks) if required_checks else False
    pass_fail['overall'] = overall

    metrics = {'files': per_file}
    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-AUD-01', metrics, thresholds_used, pass_fail)
    )

    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
