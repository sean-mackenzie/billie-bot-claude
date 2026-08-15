#!/usr/bin/env python3
"""CLI: UT-IMU-01 Sensor Nano IMU/barometer acquisition (--profile acquisition) and
UT-IMU-02 ROS IMU contract + robot_localization compatibility (--profile ekf).

Reads recorded rosbag2 data offline via BagReader, so a run can be re-scored from a copied
bag days later and produce an identical verdict.

Commanded-rotation scoring needs operator ground truth from
exports/ground_truth_segments.csv (written by `ground_truth_marker_node`, which both IMU
launch files start). If those marks are absent the rotation criteria FAIL rather than being
skipped: "clear correct-axis response with expected sign" is a required gate of test plan
section 14.8, and a run with no evidence for it has not demonstrated it.
"""

import argparse
import json
import os
import sys

import numpy as np

from billiebot_sensor_tests.common.bag_reader import BagReader
from billiebot_sensor_tests.common.config import load_bench_config
from billiebot_sensor_tests.common.ground_truth_marker import load_ground_truth_segments
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats
from billiebot_sensor_tests.common.report import (
    render_report_md, write_metrics_csv, write_metrics_json,
)
from billiebot_sensor_tests.common.result_dir import BenchResultDir
from billiebot_sensor_tests.sensor_nano.imu_metrics import (
    finite_fraction,
    mean_quaternion,
    quaternion_norms,
    score_commanded_rotation,
    segment_mask,
    stationary_stats,
    wrap_angle_rad,
    yaw_from_quaternion,
    yaw_return_error_rad,
)

#: Substrings that mark a /rosout line as a transform or sensor-timeout complaint from the
#: EKF. Matched case-insensitively against the message body.
_TF_ERROR_PATTERNS = ('transform', 'could not find', 'timeout', 'extrapolation',
                       'no matching', 'lookup would require')


def _header_stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _load_imu(bag, topic: str):
    """Return (stamps_ns, quats (N,4), gyro (N,3), accel (N,3), frame_ids, count)."""
    msgs = list(bag.messages(topic))
    if not msgs:
        return np.zeros(0, dtype=np.int64), np.zeros((0, 4)), np.zeros((0, 3)), \
            np.zeros((0, 3)), [], 0
    stamps = np.array([_header_stamp_ns(m) for _, m in msgs], dtype=np.int64)
    quats = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z]
                      for _, m in msgs], dtype=np.float64)
    gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                     for _, m in msgs], dtype=np.float64)
    accel = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                       m.linear_acceleration.z] for _, m in msgs], dtype=np.float64)
    frame_ids = sorted({m.header.frame_id for _, m in msgs})
    return stamps, quats, gyro, accel, frame_ids, len(msgs)


def _load_parser_stats(result_dir, bag, diagnostics_topic: str) -> dict:
    """Parser/peripheral counters, preferring the bridge's exports/ side-car and falling back
    to the last bagged diagnostics message.

    Two sources because the counters gate a required criterion: a truncated bag must not be
    able to quietly zero the CRC-error fraction.
    """
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
            key, value = entry.key, entry.value
            try:
                data[key] = json.loads(value)
            except ValueError:
                data[key] = value
    return data


def _segment_lookup(segments: list) -> dict:
    """label -> (t_start_ns, t_end_ns) for the FIRST occurrence of each label."""
    lookup = {}
    for segment in segments:
        lookup.setdefault(segment['label'], (segment['t_start_ns'], segment['t_end_ns']))
    return lookup


def _settled_slice(stamps, t_start_ns, t_end_ns, settle_fraction: float):
    """Mask for the settled tail of a hold segment.

    The operator is still moving the fixture at the start of a hold, so the leading
    `settle_fraction` of every marked segment is discarded before averaging orientation.
    """
    span = int(t_end_ns) - int(t_start_ns)
    settled_start = int(t_start_ns) + int(span * float(settle_fraction))
    return segment_mask(stamps, settled_start, t_end_ns)


def _score_rotations(stamps, quats, segments, cfg, prefix: str) -> dict:
    """Score every commanded rotation in the configured sequence for `prefix`.

    Returns per-step results plus an `all_correct` roll-up. Missing marks are reported
    explicitly and make `all_correct` False.
    """
    sequence = cfg.get(f'sensor_nano.{prefix}_rotation_sequence', []) or []
    reference_label = str(cfg.get(f'sensor_nano.{prefix}_reference_label', 'flat')).lower()
    settle_fraction = float(cfg.provisional(
        'sensor_nano.thresholds.provisional.rotation_segment_settle_fraction', 0.4
    ))
    min_angle_deg = float(cfg.provisional(
        f'sensor_nano.thresholds.provisional.{prefix}_min_rotation_angle_deg', 45.0
    ))
    dominance = float(cfg.provisional(
        f'sensor_nano.thresholds.provisional.{prefix}_axis_dominance_ratio', 2.0
    ))

    result = {
        'reference_label': reference_label,
        'expected_sequence': sequence,
        'min_rotation_angle_deg': min_angle_deg,
        'axis_dominance_ratio': dominance,
        'settle_fraction': settle_fraction,
        'steps': [],
        'missing_labels': [],
        'all_correct': False,
    }

    if not sequence:
        result['error'] = (
            f'sensor_nano.{prefix}_rotation_sequence is empty; nothing to score'
        )
        return result
    if not segments:
        result['error'] = (
            'exports/ground_truth_segments.csv is missing or empty -- the commanded-rotation '
            'gate cannot be evaluated. During the run, type '
            '"mark <label>" into the ground_truth_marker_node terminal at each hold.'
        )
        result['missing_labels'] = [reference_label] + [s['label'] for s in sequence]
        return result

    lookup = _segment_lookup(segments)
    if reference_label not in lookup:
        result['error'] = f'no ground-truth segment labelled {reference_label!r}'
        result['missing_labels'] = [reference_label]
        return result

    ref_start, ref_end = lookup[reference_label]
    ref_mask = _settled_slice(stamps, ref_start, ref_end, settle_fraction)
    if not np.any(ref_mask):
        result['error'] = f'no IMU samples inside the {reference_label!r} reference segment'
        return result
    q_reference = mean_quaternion(quats[ref_mask])

    all_correct = True
    for step in sequence:
        label = str(step['label']).lower()
        expected_axis = int(step['axis'])
        expected_sign = float(step['sign'])
        if label not in lookup:
            result['missing_labels'].append(label)
            result['steps'].append({'label': label, 'evaluated': False,
                                     'reason': 'no ground-truth mark with this label'})
            all_correct = False
            continue
        start, end = lookup[label]
        mask = _settled_slice(stamps, start, end, settle_fraction)
        if not np.any(mask):
            result['steps'].append({'label': label, 'evaluated': False,
                                     'reason': 'no IMU samples inside the marked segment'})
            all_correct = False
            continue
        q_held = mean_quaternion(quats[mask])
        scored = score_commanded_rotation(
            q_reference, q_held, expected_axis, expected_sign,
            np.radians(min_angle_deg), dominance,
        )
        scored.update({
            'label': label, 'evaluated': True,
            'expected_axis': expected_axis, 'expected_sign': expected_sign,
            'sample_count': int(np.sum(mask)),
        })
        step_ok = scored['angle_sufficient'] and scored['axis_correct'] and scored['sign_correct']
        scored['correct'] = bool(step_ok)
        all_correct = all_correct and step_ok
        result['steps'].append(scored)

    result['all_correct'] = bool(all_correct and result['steps'])
    return result


def _stationary_window(stamps, segments, cfg, prefix: str):
    """Mask for the settled part of the opening stationary hold.

    Falls back to the first `stationary_fallback_sec` of the recording when the operator did
    not mark it, since UT-IMU-01's script always begins with the board flat and still.
    """
    reference_label = str(cfg.get(f'sensor_nano.{prefix}_reference_label', 'flat')).lower()
    settle_fraction = float(cfg.provisional(
        'sensor_nano.thresholds.provisional.rotation_segment_settle_fraction', 0.4
    ))
    lookup = _segment_lookup(segments)
    if reference_label in lookup:
        start, end = lookup[reference_label]
        mask = _settled_slice(stamps, start, end, settle_fraction)
        if np.any(mask):
            return mask, f'ground-truth segment {reference_label!r}'

    fallback_sec = float(cfg.provisional(
        'sensor_nano.thresholds.provisional.stationary_fallback_sec', 25.0
    ))
    if stamps.size == 0:
        return np.zeros(0, dtype=bool), 'no samples'
    end_ns = int(stamps[0]) + int(fallback_sec * 1e9)
    return segment_mask(stamps, int(stamps[0]), end_ns), \
        f'first {fallback_sec:g} s of the recording (no ground-truth mark)'


def _scalar_series(bag, topic, attribute):
    """(stamps_ns, values) for a scalar field on a stamped message topic."""
    msgs = list(bag.messages(topic))
    if not msgs:
        return np.zeros(0, dtype=np.int64), np.zeros(0)
    stamps = np.array([_header_stamp_ns(m) for _, m in msgs], dtype=np.int64)
    values = np.array([getattr(m, attribute) for _, m in msgs], dtype=np.float64)
    return stamps, values


def _common_imu_checks(bag, cfg, result_dir, metrics, pass_fail, thresholds_used, prefix):
    """Message-count / frame / units / quaternion / rate / parser checks shared by both
    profiles. Returns (stamps, quats, gyro, accel) for profile-specific scoring."""
    imu_topic = cfg.get('sensor_nano.topics.imu', '/imu/data')
    expected_frame = cfg.get('sensor_nano.frames.imu', 'imu_link')

    stamps, quats, gyro, accel, frame_ids, count = _load_imu(bag, imu_topic)
    metrics['imu_message_count'] = count
    metrics['imu_topic'] = imu_topic
    metrics['imu_frame_ids'] = frame_ids
    metrics['imu_message_type'] = bag.topic_names_and_types.get(imu_topic, '')

    pass_fail['imu_present'] = count > 0
    thresholds_used['imu_present'] = {'value': count, 'threshold': '> 0', 'tier': 'required'}

    pass_fail['imu_message_type_correct'] = (
        metrics['imu_message_type'] == 'sensor_msgs/msg/Imu'
    )
    thresholds_used['imu_message_type_correct'] = {
        'value': metrics['imu_message_type'] or '(topic not in bag)',
        'threshold': 'sensor_msgs/msg/Imu', 'tier': 'required'}

    pass_fail['imu_frame_id_correct'] = frame_ids == [expected_frame]
    thresholds_used['imu_frame_id_correct'] = {
        'value': frame_ids, 'threshold': [expected_frame], 'tier': 'required'}

    if count == 0:
        return stamps, quats, gyro, accel

    components = np.hstack([quats, gyro, accel])
    finite = finite_fraction(components)
    min_finite = cfg.required(f'sensor_nano.thresholds.required.{prefix}_min_finite_fraction')
    metrics['imu_finite_fraction'] = finite
    pass_fail['imu_finite_fraction'] = finite >= min_finite
    thresholds_used['imu_finite_fraction'] = {
        'value': finite, 'threshold': f'>= {min_finite}', 'tier': 'required'}

    norms = quaternion_norms(quats)
    norm_min = cfg.required(f'sensor_nano.thresholds.required.{prefix}_quat_norm_min')
    norm_max = cfg.required(f'sensor_nano.thresholds.required.{prefix}_quat_norm_max')
    min_fraction = cfg.required(
        f'sensor_nano.thresholds.required.{prefix}_min_quat_norm_fraction'
    )
    in_band = float(np.mean((norms >= norm_min) & (norms <= norm_max)))
    metrics['quaternion_norm'] = {
        'in_band_fraction': in_band, 'mean': float(np.mean(norms)),
        'min': float(np.min(norms)), 'max': float(np.max(norms)),
        'band': [norm_min, norm_max],
    }
    pass_fail['quaternion_norm_in_band'] = in_band >= min_fraction
    thresholds_used['quaternion_norm_in_band'] = {
        'value': in_band, 'threshold': f'>= {min_fraction} within {norm_min}-{norm_max}',
        'tier': 'required'}

    stats = compute_rate_stats(stamps)
    metrics['imu_rate'] = stats.to_dict()
    min_rate = cfg.required(f'sensor_nano.thresholds.required.{prefix}_min_imu_rate_hz')
    max_gap = cfg.required(f'sensor_nano.thresholds.required.{prefix}_max_imu_gap_s')
    pass_fail['imu_rate'] = stats.mean_hz >= min_rate
    pass_fail['imu_max_gap'] = stats.max_gap_s <= max_gap
    pass_fail['imu_timestamps_monotonic'] = stats.monotonic
    thresholds_used['imu_rate'] = {
        'value': stats.mean_hz, 'threshold': f'>= {min_rate} Hz', 'tier': 'required'}
    thresholds_used['imu_max_gap'] = {
        'value': stats.max_gap_s, 'threshold': f'<= {max_gap} s', 'tier': 'required'}
    thresholds_used['imu_timestamps_monotonic'] = {
        'value': stats.monotonic, 'threshold': True, 'tier': 'required'}

    parser = _load_parser_stats(
        result_dir, bag, cfg.get('sensor_nano.topics.diagnostics',
                                  '/bench/sensor_nano/diagnostics')
    )
    metrics['parser_stats'] = parser
    metrics['firmware_error_counters'] = _firmware_error_counters(parser)

    max_parse_error = cfg.required(
        f'sensor_nano.thresholds.required.{prefix}_max_parse_error_fraction'
    )
    parse_fraction = _as_float(parser.get('parse_error_fraction'))
    metrics['parse_error_fraction'] = parse_fraction
    # An unavailable counter is a failure, not a pass: the required criterion is "CRC/parser
    # failure fraction <= 0.1%", and "we could not tell" does not satisfy it.
    pass_fail['parse_error_fraction'] = (
        parse_fraction is not None and parse_fraction <= max_parse_error
    )
    thresholds_used['parse_error_fraction'] = {
        'value': parse_fraction if parse_fraction is not None else 'unavailable',
        'threshold': f'<= {max_parse_error}', 'tier': 'required'}

    max_seq_gap = cfg.required(
        f'sensor_nano.thresholds.required.{prefix}_max_sequence_discontinuity_fraction'
    )
    seq_fraction = _as_float(parser.get('sequence_discontinuity_fraction'))
    metrics['sequence_discontinuity_fraction'] = seq_fraction
    pass_fail['sequence_discontinuity_fraction'] = (
        seq_fraction is not None and seq_fraction <= max_seq_gap
    )
    thresholds_used['sequence_discontinuity_fraction'] = {
        'value': seq_fraction if seq_fraction is not None else 'unavailable',
        'threshold': f'<= {max_seq_gap}', 'tier': 'required'}

    return stamps, quats, gyro, accel


def _as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(value):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


#: Firmware health counters carried by the Nano's S record, recorded for every IMU profile.
#:
#: Recorded, never gated. The required criteria (rate, sequence continuity, CRC/parser error
#: fraction, stationary acceleration, gyro plausibility) stay authoritative -- a single
#: transient recovery is not by itself a failure and no threshold here would be anything but
#: invented. They are surfaced by name because the firmware drops an entire IMU sample when
#: any BNO055 transaction fails, and a dropped sample consumes no sequence number: an
#: intermittent I2C fault shows up as a small rate deficit plus `imu_read_errors`, and
#: nowhere else.
_FIRMWARE_ERROR_COUNTERS = (
    'imu_read_errors', 'i2c_errors', 'bmp_errors', 'reinits', 'imu_records_dropped',
)


def _firmware_error_counters(parser: dict) -> dict:
    """Pull the S-record counters out of whichever parser-stats shape we were handed.

    The exports side-car nests them under `last_status`; the bagged diagnostics fallback
    flattens them into top-level keys. Same two-source pattern as the bno_ok lookup below.
    """
    nested = parser.get('last_status')
    nested = nested if isinstance(nested, dict) else {}
    counters = {}
    for key in _FIRMWARE_ERROR_COUNTERS:
        value = _as_int(parser.get(key))
        if value is None:
            value = _as_int(nested.get(key))
        counters[key] = value
    return counters


def _run_acquisition_profile(bag, cfg, result_dir) -> dict:
    metrics = {'profile': 'acquisition'}
    pass_fail = {}
    thresholds_used = {}

    stamps, quats, gyro, accel = _common_imu_checks(
        bag, cfg, result_dir, metrics, pass_fail, thresholds_used, 'ut_imu_01'
    )

    parser = metrics.get('parser_stats', {})
    bno_ok = _as_int(parser.get('bno_ok'))
    bmp_ok = _as_int(parser.get('bmp_ok'))
    if bno_ok is None and isinstance(parser.get('last_status'), dict):
        bno_ok = _as_int(parser['last_status'].get('bno_ok'))
        bmp_ok = _as_int(parser['last_status'].get('bmp_ok'))
    metrics['bno055_initialized'] = bno_ok
    metrics['bmp280_initialized'] = bmp_ok
    pass_fail['bno055_initialized'] = bno_ok == 1
    pass_fail['bmp280_initialized'] = bmp_ok == 1
    thresholds_used['bno055_initialized'] = {
        'value': bno_ok if bno_ok is not None else 'unavailable',
        'threshold': 1, 'tier': 'required'}
    thresholds_used['bmp280_initialized'] = {
        'value': bmp_ok if bmp_ok is not None else 'unavailable',
        'threshold': 1, 'tier': 'required'}

    # Calibration is recorded, never gated (test plan 14.8: "A magnetometer calibration
    # value below 3 does not by itself fail this hardware-acquisition test").
    metrics['calibration_informational'] = {
        key: _as_int(parser.get(key)) for key in ('cal_sys', 'cal_gyr', 'cal_acc', 'cal_mag')
    }
    if isinstance(parser.get('last_calibration'), dict):
        metrics['calibration_informational'] = parser['last_calibration']

    segments = load_ground_truth_segments(result_dir)
    metrics['ground_truth_segment_count'] = len(segments)

    if stamps.size:
        mask, source = _stationary_window(stamps, segments, cfg, 'ut_imu_01')
        metrics['stationary_window_source'] = source
        accel_stats = stationary_stats(accel[mask])
        gyro_stats = stationary_stats(gyro[mask])
        metrics['stationary_acceleration'] = accel_stats
        metrics['stationary_gyro'] = gyro_stats

        accel_min = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_imu_01_stationary_accel_min_mps2', 8.5)
        accel_max = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_imu_01_stationary_accel_max_mps2', 11.2)
        gyro_max = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_imu_01_max_stationary_gyro_rad_s', 0.10)

        pass_fail['stationary_acceleration_magnitude'] = (
            accel_stats['count'] > 0 and accel_min <= accel_stats['mean'] <= accel_max
        )
        pass_fail['stationary_gyro_magnitude'] = (
            gyro_stats['count'] > 0 and gyro_stats['mean'] <= gyro_max
        )
        thresholds_used['stationary_acceleration_magnitude'] = {
            'value': accel_stats['mean'], 'threshold': f'{accel_min}-{accel_max} m/s^2',
            'tier': 'provisional'}
        thresholds_used['stationary_gyro_magnitude'] = {
            'value': gyro_stats['mean'], 'threshold': f'<= {gyro_max} rad/s',
            'tier': 'provisional'}

        rotations = _score_rotations(stamps, quats, segments, cfg, 'ut_imu_01')
        metrics['commanded_rotations'] = rotations
        pass_fail['commanded_rotation_response'] = rotations['all_correct']
        thresholds_used['commanded_rotation_response'] = {
            'value': rotations.get('error', f"{sum(1 for s in rotations['steps'] if s.get('correct'))}"
                                             f"/{len(rotations['steps'])} steps correct"),
            'threshold': 'every commanded rotation on the correct axis with the expected sign',
            'tier': 'provisional'}

    _score_barometer(bag, cfg, metrics, pass_fail, thresholds_used)

    overall = all(v for k, v in pass_fail.items() if k != 'overall')
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-IMU-01', metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _score_barometer(bag, cfg, metrics, pass_fail, thresholds_used) -> None:
    pressure_topic = cfg.get('sensor_nano.topics.pressure', '/barometer/pressure')
    temperature_topic = cfg.get('sensor_nano.topics.temperature', '/barometer/temperature')

    p_stamps, pressures = _scalar_series(bag, pressure_topic, 'fluid_pressure')
    t_stamps, temperatures = _scalar_series(bag, temperature_topic, 'temperature')

    p_min = cfg.required('sensor_nano.thresholds.required.ut_imu_01_pressure_min_pa')
    p_max = cfg.required('sensor_nano.thresholds.required.ut_imu_01_pressure_max_pa')
    t_min = cfg.provisional(
        'sensor_nano.thresholds.provisional.ut_imu_01_temperature_min_c', 5.0)
    t_max = cfg.provisional(
        'sensor_nano.thresholds.provisional.ut_imu_01_temperature_max_c', 45.0)

    metrics['barometer'] = {
        'pressure_count': int(pressures.size),
        'temperature_count': int(temperatures.size),
        'pressure_mean_pa': float(np.mean(pressures)) if pressures.size else None,
        'pressure_min_pa': float(np.min(pressures)) if pressures.size else None,
        'pressure_max_pa': float(np.max(pressures)) if pressures.size else None,
        'temperature_mean_c': float(np.mean(temperatures)) if temperatures.size else None,
        'temperature_min_c': float(np.min(temperatures)) if temperatures.size else None,
        'temperature_max_c': float(np.max(temperatures)) if temperatures.size else None,
        'pressure_rate': compute_rate_stats(p_stamps).to_dict() if p_stamps.size else None,
    }

    pressure_ok = bool(
        pressures.size
        and np.all(np.isfinite(pressures))
        and np.all((pressures >= p_min) & (pressures <= p_max))
    )
    temperature_ok = bool(
        temperatures.size
        and np.all(np.isfinite(temperatures))
        and np.all((temperatures >= t_min) & (temperatures <= t_max))
    )
    pass_fail['pressure_plausible'] = pressure_ok
    pass_fail['temperature_plausible'] = temperature_ok
    thresholds_used['pressure_plausible'] = {
        'value': metrics['barometer']['pressure_mean_pa'],
        'threshold': f'all samples finite within {p_min}-{p_max} Pa', 'tier': 'required'}
    thresholds_used['temperature_plausible'] = {
        'value': metrics['barometer']['temperature_mean_c'],
        'threshold': f'all samples finite within {t_min}-{t_max} C', 'tier': 'provisional'}


def _run_ekf_profile(bag, cfg, result_dir) -> dict:
    metrics = {'profile': 'ekf'}
    pass_fail = {}
    thresholds_used = {}

    stamps, quats, _gyro, _accel = _common_imu_checks(
        bag, cfg, result_dir, metrics, pass_fail, thresholds_used, 'ut_imu_02'
    )

    odom_topic = cfg.get('sensor_nano.topics.filtered_odometry', '/odometry/filtered')
    odom_msgs = list(bag.messages(odom_topic))
    metrics['odometry_message_count'] = len(odom_msgs)
    metrics['odometry_topic'] = odom_topic

    pass_fail['filtered_odometry_published'] = len(odom_msgs) > 0
    thresholds_used['filtered_odometry_published'] = {
        'value': len(odom_msgs), 'threshold': '> 0', 'tier': 'required'}

    if odom_msgs:
        odom_stamps = np.array([_header_stamp_ns(m) for _, m in odom_msgs], dtype=np.int64)
        odom_quats = np.array(
            [[m.pose.pose.orientation.w, m.pose.pose.orientation.x,
              m.pose.pose.orientation.y, m.pose.pose.orientation.z] for _, m in odom_msgs],
            dtype=np.float64)
        odom_values = np.array(
            [[m.pose.pose.position.x, m.pose.pose.position.y,
              m.twist.twist.linear.x, m.twist.twist.angular.z] for _, m in odom_msgs],
            dtype=np.float64)

        odom_stats = compute_rate_stats(odom_stamps)
        metrics['odometry_rate'] = odom_stats.to_dict()
        min_ekf_rate = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_imu_02_min_ekf_rate_hz', 27.0)
        pass_fail['ekf_rate'] = odom_stats.mean_hz >= min_ekf_rate
        thresholds_used['ekf_rate'] = {
            'value': odom_stats.mean_hz, 'threshold': f'>= {min_ekf_rate} Hz',
            'tier': 'provisional'}

        odom_finite = finite_fraction(np.hstack([odom_quats, odom_values]))
        metrics['odometry_finite_fraction'] = odom_finite
        pass_fail['ekf_output_finite'] = odom_finite >= 1.0
        thresholds_used['ekf_output_finite'] = {
            'value': odom_finite, 'threshold': '100%', 'tier': 'required'}

        segments = load_ground_truth_segments(result_dir)
        metrics['ground_truth_segment_count'] = len(segments)

        rotations = _score_rotations(stamps, quats, segments, cfg, 'ut_imu_02')
        metrics['commanded_rotations'] = rotations
        pass_fail['imu_yaw_response'] = rotations['all_correct']
        thresholds_used['imu_yaw_response'] = {
            'value': rotations.get('error', f"{sum(1 for s in rotations['steps'] if s.get('correct'))}"
                                             f"/{len(rotations['steps'])} steps correct"),
            'threshold': 'yaw follows each commanded rotation with the expected sign',
            'tier': 'provisional'}

        yaw_agreement = _score_yaw_agreement(stamps, quats, odom_stamps, odom_quats, segments,
                                              cfg)
        metrics['ekf_yaw_agreement'] = yaw_agreement
        pass_fail['ekf_yaw_sign_matches_imu'] = yaw_agreement.get('sign_matches', False)
        thresholds_used['ekf_yaw_sign_matches_imu'] = {
            'value': yaw_agreement.get('correlation'),
            'threshold': 'EKF yaw moves with the same sign as IMU yaw',
            'tier': 'provisional'}

        return_error = _score_yaw_return(stamps, quats, segments, cfg)
        metrics['yaw_return'] = return_error
        max_return_deg = cfg.provisional(
            'sensor_nano.thresholds.provisional.ut_imu_02_max_yaw_return_error_deg', 15.0)
        pass_fail['yaw_returns_to_start'] = (
            return_error.get('error_deg') is not None
            and return_error['error_deg'] <= max_return_deg
        )
        thresholds_used['yaw_returns_to_start'] = {
            'value': return_error.get('error_deg', 'unavailable'),
            'threshold': f'<= {max_return_deg} deg', 'tier': 'provisional'}

    tf_result = _score_tf_errors(bag, cfg)
    metrics['tf_errors'] = tf_result
    pass_fail['no_sustained_tf_errors'] = tf_result['within_limit']
    thresholds_used['no_sustained_tf_errors'] = {
        'value': tf_result['matching_message_count'],
        'threshold': f"<= {tf_result['limit']} transform/timeout log messages",
        'tier': 'provisional'}

    overall = all(v for k, v in pass_fail.items() if k != 'overall')
    pass_fail['overall'] = overall

    write_metrics_json(result_dir.metrics_json_path, metrics, pass_fail)
    write_metrics_csv(result_dir.metrics_csv_path, metrics)
    result_dir.report_path.write_text(
        render_report_md('UT-IMU-02', metrics, thresholds_used, pass_fail)
    )
    return {'pass': overall}


def _score_yaw_agreement(imu_stamps, imu_quats, odom_stamps, odom_quats, segments, cfg) -> dict:
    """Does the EKF's yaw move in the same direction as the IMU's, over the marked holds?

    Compares *changes* between consecutive holds rather than absolute yaw, so an arbitrary
    EKF yaw origin (it starts at zero, the IMU does not) cannot make a correctly-signed
    filter look wrong.
    """
    settle_fraction = float(cfg.provisional(
        'sensor_nano.thresholds.provisional.rotation_segment_settle_fraction', 0.4))
    if len(segments) < 2:
        return {'evaluated': False,
                'reason': 'need at least two ground-truth marks to compare yaw changes'}

    imu_yaws, odom_yaws = [], []
    for segment in segments:
        imu_mask = _settled_slice(imu_stamps, segment['t_start_ns'], segment['t_end_ns'],
                                   settle_fraction)
        odom_mask = _settled_slice(odom_stamps, segment['t_start_ns'], segment['t_end_ns'],
                                    settle_fraction)
        if not np.any(imu_mask) or not np.any(odom_mask):
            continue
        imu_yaws.append(float(yaw_from_quaternion(mean_quaternion(imu_quats[imu_mask]))[0]))
        odom_yaws.append(float(yaw_from_quaternion(mean_quaternion(odom_quats[odom_mask]))[0]))

    if len(imu_yaws) < 2:
        return {'evaluated': False,
                'reason': 'fewer than two segments had both IMU and EKF samples'}

    imu_deltas = wrap_angle_rad(np.diff(np.array(imu_yaws)))
    odom_deltas = wrap_angle_rad(np.diff(np.array(odom_yaws)))

    min_delta = np.radians(float(cfg.provisional(
        'sensor_nano.thresholds.provisional.ut_imu_02_min_yaw_delta_deg', 20.0)))
    significant = np.abs(imu_deltas) >= min_delta
    if not np.any(significant):
        return {'evaluated': False,
                'reason': f'no segment-to-segment yaw change exceeded {np.degrees(min_delta):g} deg'}

    agree = np.sign(imu_deltas[significant]) == np.sign(odom_deltas[significant])
    return {
        'evaluated': True,
        'imu_yaw_deltas_deg': [float(v) for v in np.degrees(imu_deltas)],
        'ekf_yaw_deltas_deg': [float(v) for v in np.degrees(odom_deltas)],
        'significant_transitions': int(np.sum(significant)),
        'agreeing_transitions': int(np.sum(agree)),
        'correlation': float(np.mean(agree)),
        'sign_matches': bool(np.all(agree)),
    }


def _score_yaw_return(stamps, quats, segments, cfg) -> dict:
    """Yaw error between the first and last marked holds."""
    settle_fraction = float(cfg.provisional(
        'sensor_nano.thresholds.provisional.rotation_segment_settle_fraction', 0.4))
    lookup = _segment_lookup(segments)
    start_label = str(cfg.get('sensor_nano.ut_imu_02_reference_label', 'flat')).lower()
    end_label = str(cfg.get('sensor_nano.ut_imu_02_return_label', 'flat_end')).lower()

    if start_label not in lookup or end_label not in lookup:
        return {'evaluated': False, 'error_deg': None,
                'reason': f'need ground-truth marks {start_label!r} and {end_label!r}'}

    start_mask = _settled_slice(stamps, *lookup[start_label], settle_fraction)
    end_mask = _settled_slice(stamps, *lookup[end_label], settle_fraction)
    if not np.any(start_mask) or not np.any(end_mask):
        return {'evaluated': False, 'error_deg': None,
                'reason': 'no IMU samples inside one of the marked holds'}

    error_rad = yaw_return_error_rad(
        mean_quaternion(quats[start_mask]), mean_quaternion(quats[end_mask])
    )
    return {'evaluated': True, 'error_rad': error_rad,
            'error_deg': float(np.degrees(error_rad)),
            'start_label': start_label, 'end_label': end_label}


def _score_tf_errors(bag, cfg) -> dict:
    """Count transform/timeout complaints on the bagged /rosout stream.

    /rosout is recorded (rather than grepping console.log) because console.log holds only
    preflight output unless ROS_LOG_DIR is redirected, and because the bag is this suite's
    authoritative evidence.
    """
    rosout_topic = cfg.get('sensor_nano.topics.rosout', '/rosout')
    limit = int(cfg.provisional(
        'sensor_nano.thresholds.provisional.ut_imu_02_max_tf_error_messages', 10))

    messages = list(bag.messages(rosout_topic))
    matching = []
    for _stamp, msg in messages:
        level = getattr(msg, 'level', 0)
        text = str(getattr(msg, 'msg', ''))
        lowered = text.lower()
        # WARN=30, ERROR=40, FATAL=50 in rcl_interfaces/msg/Log.
        if level >= 30 and any(pattern in lowered for pattern in _TF_ERROR_PATTERNS):
            matching.append({'level': int(level), 'name': str(getattr(msg, 'name', '')),
                              'msg': text})

    return {
        'rosout_recorded': len(messages) > 0,
        'rosout_message_count': len(messages),
        'matching_message_count': len(matching),
        'limit': limit,
        'within_limit': len(matching) <= limit,
        'examples': matching[:10],
    }


def _default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def _self_test() -> int:
    """Offline check of the rotation-scoring math with no bag and no hardware."""
    from billiebot_sensor_tests.sensor_nano.imu_metrics import score_commanded_rotation

    half = np.sqrt(0.5)
    identity = np.array([[1.0, 0.0, 0.0, 0.0]])
    x_plus_90 = np.array([[half, half, 0.0, 0.0]])
    scored = score_commanded_rotation(identity, x_plus_90, 0, 1.0, np.radians(45.0))
    print(f"[self-test] +90 deg about X -> axis={scored['dominant_axis_index']} "
          f"sign={scored['dominant_axis_sign']:+.0f} "
          f"angle={scored['measured_angle_deg']:.1f} deg")
    ok = scored['axis_correct'] and scored['sign_correct'] and scored['angle_sufficient']
    print('[self-test] PASS' if ok else '[self-test] FAIL')
    return 0 if ok else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Score a Sensor Nano IMU bench run (UT-IMU-01 / UT-IMU-02).'
    )
    parser.add_argument('--results-dir')
    parser.add_argument('--config-file', default='')
    parser.add_argument('--profile', choices=['acquisition', 'ekf'], default='acquisition')
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

    if args.profile == 'acquisition':
        result = _run_acquisition_profile(bag, cfg, result_dir)
    else:
        result = _run_ekf_profile(bag, cfg, result_dir)

    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
