"""Synthetic Sensor Nano telemetry fixtures: serial records, quaternion holds, PSU sweeps
and mission-status series.

Deterministic factories only -- no @pytest.fixture decorators, matching the other fixture
modules in this directory. Every record is built through the production
`protocol.build_line()`, so a fixture can never encode a framing the firmware would not
actually emit.
"""

import numpy as np

from billiebot_sensor_tests.sensor_nano.protocol import build_line

#: Matches the firmware's IMU_PERIOD_US / BATTERY_PERIOD_US etc.
IMU_PERIOD_US = 20000
BATTERY_PERIOD_US = 200000


def imu_line(seq: int, t_us: int, quat=(1.0, 0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0),
             accel=(0.0, 0.0, 9.81), calibration=(3, 3, 3, 3)) -> str:
    """One `I` record with the same field order and decimal precision as the firmware."""
    fields = [seq, t_us]
    fields += [f'{v:.5f}' for v in quat]
    fields += [f'{v:.4f}' for v in gyro]
    fields += [f'{v:.3f}' for v in accel]
    fields += list(calibration)
    return build_line('I', fields)


def battery_line(seq: int, t_us: int, adc_mean: float) -> str:
    return build_line('B', [seq, t_us, f'{adc_mean:.2f}'])


def barometer_line(seq: int, t_us: int, pressure_pa=101325.0, temperature_c=21.5) -> str:
    return build_line('P', [seq, t_us, f'{pressure_pa:.1f}', f'{temperature_c:.2f}'])


def magnetometer_line(seq: int, t_us: int, field_ut=(20.0, -5.0, 40.0)) -> str:
    return build_line('M', [seq, t_us] + [f'{v:.3f}' for v in field_ut])


def status_line(seq: int, t_us: int, bno_ok=1, bmp_ok=1, fusion_mode=12, i2c_errors=0,
                 imu_read_errors=0, bmp_errors=0, reinits=0, imu_records_dropped=0) -> str:
    return build_line('S', [seq, t_us, bno_ok, bmp_ok, fusion_mode, i2c_errors,
                             imu_read_errors, bmp_errors, reinits, imu_records_dropped])


def corrupt_crc(line: str) -> str:
    """Flip the CRC so the payload stays well-formed but the checksum fails.

    Uses the payload's own characters rather than a fixed replacement, so the result is a
    genuine CRC mismatch rather than an accidentally-valid alternative framing.
    """
    payload, _sep, crc = line.rpartition('*')
    flipped = f'{(int(crc, 16) ^ 0x0001):04X}'
    return f'{payload}*{flipped}'


def imu_stream(count: int, start_seq: int = 0, start_t_us: int = 0,
                period_us: int = IMU_PERIOD_US) -> list:
    """`count` consecutive stationary IMU records with monotonic seq and micros."""
    return [
        imu_line(start_seq + i, (start_t_us + i * period_us) % (1 << 32))
        for i in range(count)
    ]


def quaternion_hold(quat, count: int = 50, noise_std: float = 0.0, seed: int = 0):
    """`count` samples of a held orientation, optionally with a little sensor noise."""
    base = np.tile(np.asarray(quat, dtype=np.float64), (count, 1))
    if noise_std > 0:
        rng = np.random.default_rng(seed)
        base = base + rng.normal(0.0, noise_std, size=base.shape)
        base = base / np.linalg.norm(base, axis=1, keepdims=True)
    return base


def axis_quaternion(axis_index: int, angle_deg: float) -> np.ndarray:
    """Unit quaternion for `angle_deg` about the given body axis (0=X, 1=Y, 2=Z)."""
    half = np.radians(angle_deg) / 2.0
    quat = np.zeros(4)
    quat[0] = np.cos(half)
    quat[1 + axis_index] = np.sin(half)
    return quat.reshape(1, 4)


def timestamps_ns(count: int, hz: float, start_ns: int = 0) -> np.ndarray:
    period_ns = int(1e9 / hz)
    return np.array([start_ns + i * period_ns for i in range(count)], dtype=np.int64)


def battery_sweep(divider_ratio: float = 6.0, adc_reference: float = 5.0,
                   voltages=(9.9, 10.3, 10.5, 10.7, 11.1, 12.0, 12.6),
                   voltage_error: float = 0.0, adc_max_count: float = 1023.0):
    """A synthetic PSU sweep as (dmm_battery_v, dmm_a0_v, adc_counts, ros_voltage).

    `voltage_error` adds a constant bias to the ROS-reported voltage, which is how the tests
    exercise the accuracy gates without needing a real divider.
    """
    dmm_battery = np.asarray(voltages, dtype=np.float64)
    dmm_a0 = dmm_battery / divider_ratio
    adc = np.round(dmm_a0 / adc_reference * adc_max_count)
    ros_voltage = adc / adc_max_count * adc_reference * divider_ratio + voltage_error
    return dmm_battery, dmm_a0, adc, ros_voltage


def mission_status_series(count: int, hz: float = 2.0, mode: int = 1,
                           safe_from_index: int = None, start_ns: int = 0):
    """(stamps_ns, modes) for a /billiebot/mission_status stream that optionally enters SAFE.

    `safe_from_index` models the production controller's latching behaviour: once SAFE is
    entered it is never left (mission_controller.py has no exit path other than /set_mode).
    """
    stamps = timestamps_ns(count, hz, start_ns)
    modes = np.full(count, int(mode), dtype=np.int64)
    if safe_from_index is not None:
        modes[safe_from_index:] = 5
    return stamps, modes


def battery_voltage_series(count: int, hz: float = 5.0, high_v: float = 10.70,
                            low_v: float = 10.30, step_index: int = None, start_ns: int = 0):
    """(stamps_ns, voltages) stepping from `high_v` to `low_v` at `step_index`."""
    stamps = timestamps_ns(count, hz, start_ns)
    voltages = np.full(count, float(high_v), dtype=np.float64)
    if step_index is not None:
        voltages[step_index:] = float(low_v)
    return stamps, voltages
