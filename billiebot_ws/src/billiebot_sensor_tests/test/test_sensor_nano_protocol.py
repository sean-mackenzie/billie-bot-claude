"""Sensor Nano wire-protocol tests: CRC, strict parsing, line reassembly, and stream
integrity across sequence and micros() rollover.

These lock in the contract shared with firmware/sensor_nano/sensor_nano.ino. The CRC check
vector in particular is asserted in both places, so the two implementations cannot drift
apart without a test failing here.
"""

import pytest

from fixtures.sensor_nano_fixtures import (
    barometer_line,
    battery_line,
    corrupt_crc,
    imu_line,
    imu_stream,
    magnetometer_line,
    status_line,
)

from billiebot_sensor_tests.sensor_nano.protocol import (
    CRC16_CHECK_INPUT,
    CRC16_CHECK_VALUE,
    UINT32_MODULUS,
    BarometerRecord,
    BatteryRecord,
    CrcError,
    ImuRecord,
    MagnetometerRecord,
    MalformedRecordError,
    SerialLineAssembler,
    StatusRecord,
    StreamStats,
    build_line,
    crc16_ccitt_false,
    parse_line,
)


# --- CRC ---------------------------------------------------------------------------------

def test_crc16_matches_the_canonical_ccitt_false_check_value():
    # 0x29B1 for "123456789" is the published CRC-16/CCITT-FALSE check value, and the
    # firmware asserts the same constant at boot.
    assert crc16_ccitt_false(CRC16_CHECK_INPUT) == CRC16_CHECK_VALUE == 0x29B1


def test_crc16_is_order_sensitive():
    assert crc16_ccitt_false(b'AB') != crc16_ccitt_false(b'BA')


def test_crc16_of_empty_input_is_the_init_value():
    assert crc16_ccitt_false(b'') == 0xFFFF


def test_build_line_round_trips_through_parse_line():
    line = build_line('B', [7, 1234, '512.25'])
    record = parse_line(line)
    assert isinstance(record, BatteryRecord)
    assert (record.seq, record.t_us, record.adc_mean) == (7, 1234, 512.25)


# --- record types ------------------------------------------------------------------------

def test_imu_record_parses_every_field_in_order():
    line = imu_line(3, 40000, quat=(0.70711, 0.70711, 0.0, 0.0),
                    gyro=(0.1, -0.2, 0.3), accel=(0.5, -0.25, 9.81),
                    calibration=(3, 3, 2, 1))
    record = parse_line(line)
    assert isinstance(record, ImuRecord)
    assert record.seq == 3 and record.t_us == 40000
    assert record.quaternion == pytest.approx((0.70711, 0.70711, 0.0, 0.0))
    assert record.angular_velocity == pytest.approx((0.1, -0.2, 0.3))
    assert record.linear_acceleration == pytest.approx((0.5, -0.25, 9.81))
    assert (record.cal_sys, record.cal_gyr, record.cal_acc, record.cal_mag) == (3, 3, 2, 1)


def test_battery_record_carries_raw_adc_not_volts():
    # The firmware must never send volts: the divider ratio and ADC reference live on the
    # Jetson so they can be recalibrated without a reflash.
    record = parse_line(battery_line(1, 100, 341.75))
    assert record.adc_mean == pytest.approx(341.75)
    assert not hasattr(record, 'voltage')


def test_barometer_record_parses_pressure_and_temperature():
    record = parse_line(barometer_line(2, 200, 99512.4, 22.75))
    assert isinstance(record, BarometerRecord)
    assert record.pressure_pa == pytest.approx(99512.4)
    assert record.temperature_c == pytest.approx(22.75)


def test_magnetometer_record_parses_three_axes():
    record = parse_line(magnetometer_line(4, 400, (20.5, -5.25, 40.125)))
    assert isinstance(record, MagnetometerRecord)
    assert record.magnetic_field == pytest.approx((20.5, -5.25, 40.125))


def test_status_record_parses_health_counters():
    line = status_line(5, 500, bno_ok=1, bmp_ok=0, fusion_mode=12, i2c_errors=7,
                       imu_read_errors=2, bmp_errors=9, reinits=1, imu_records_dropped=4)
    record = parse_line(line)
    assert isinstance(record, StatusRecord)
    assert (record.bno_ok, record.bmp_ok, record.fusion_mode) == (1, 0, 12)
    assert (record.i2c_errors, record.imu_read_errors) == (7, 2)
    assert (record.bmp_errors, record.reinits, record.imu_records_dropped) == (9, 1, 4)


# --- rejection paths ---------------------------------------------------------------------

def test_crc_failure_raises_crc_error():
    with pytest.raises(CrcError):
        parse_line(corrupt_crc(imu_line(1, 100)))


def test_missing_crc_delimiter_is_malformed():
    with pytest.raises(MalformedRecordError):
        parse_line('I,1,100,1.0,0.0,0.0,0.0,0,0,0,0,0,9.81,3,3,3,3')


def test_non_hex_crc_is_malformed():
    with pytest.raises(MalformedRecordError):
        parse_line('B,1,100,512.0*ZZZZ')


def test_short_crc_field_is_malformed():
    with pytest.raises(MalformedRecordError):
        parse_line(build_line('B', [1, 100, '512.0'])[:-2])


def test_unknown_record_type_is_malformed():
    with pytest.raises(MalformedRecordError):
        parse_line(build_line('X', [1, 100, '0.0']))


def test_wrong_field_count_is_malformed():
    with pytest.raises(MalformedRecordError):
        parse_line(build_line('P', [1, 100, '101325.0']))  # missing temperature


def test_non_numeric_field_is_malformed_not_zero():
    # The whole point: a bad field must never become 0.0. A silently-zeroed quaternion or
    # ADC count is indistinguishable from real data downstream.
    with pytest.raises(MalformedRecordError):
        parse_line(build_line('B', [1, 100, 'not_a_number']))


def test_non_finite_float_is_rejected():
    for bad in ('nan', 'inf', '-inf'):
        with pytest.raises(MalformedRecordError):
            parse_line(build_line('B', [1, 100, bad]))


def test_negative_sequence_is_out_of_uint32_range():
    with pytest.raises(MalformedRecordError):
        parse_line(build_line('B', [-1, 100, '512.0']))


def test_empty_line_is_malformed():
    with pytest.raises(MalformedRecordError):
        parse_line('   ')


# --- line assembly -----------------------------------------------------------------------

def test_assembler_reassembles_a_record_split_across_two_reads():
    line = imu_line(1, 100)
    assembler = SerialLineAssembler()
    first = assembler.feed(line[:20].encode('ascii'))
    assert first == []
    second = assembler.feed((line[20:] + '\n').encode('ascii'))
    assert second == [line]
    assert parse_line(second[0]).seq == 1


def test_assembler_returns_multiple_complete_lines_from_one_chunk():
    lines = imu_stream(3)
    chunk = ('\n'.join(lines) + '\n').encode('ascii')
    assert SerialLineAssembler().feed(chunk) == lines


def test_assembler_holds_a_trailing_partial_line():
    lines = imu_stream(2)
    chunk = (lines[0] + '\n' + lines[1][:10]).encode('ascii')
    assembler = SerialLineAssembler()
    assert assembler.feed(chunk) == [lines[0]]
    assert assembler.feed((lines[1][10:] + '\n').encode('ascii')) == [lines[1]]


def test_assembler_discards_a_buffer_that_never_terminates():
    assembler = SerialLineAssembler(max_line_length=64)
    assert assembler.feed(b'x' * 200) == []
    assert assembler.overlong_discards == 1
    # Resynchronises on the next complete line rather than staying wedged.
    line = imu_line(1, 100)
    assert assembler.feed((line + '\n').encode('ascii')) == [line]


def test_assembler_tolerates_non_ascii_noise_without_raising():
    assembler = SerialLineAssembler()
    lines = assembler.feed(b'\xff\xfe garbage\n')
    assert len(lines) == 1  # decoded with replacement, then rejected by parse_line
    with pytest.raises(MalformedRecordError):
        parse_line(lines[0])


# --- stream statistics -------------------------------------------------------------------

def test_stream_stats_counts_records_by_type():
    stats = StreamStats()
    for line in imu_stream(4):
        stats.note_record(parse_line(line))
    stats.note_record(parse_line(battery_line(4, 100, 500.0)))
    assert stats.type_counts['I'] == 4
    assert stats.type_counts['B'] == 1
    assert stats.records_ok == 5
    assert stats.parse_error_fraction == 0.0


def test_stream_stats_reports_error_fractions():
    stats = StreamStats()
    for line in imu_stream(8):
        stats.note_record(parse_line(line))
    stats.note_crc_error()
    stats.note_malformed()
    assert stats.crc_errors == 1
    assert stats.malformed_errors == 1
    assert stats.parse_error_count == 2
    assert stats.parse_error_fraction == pytest.approx(2 / 10)


def test_empty_stream_reports_zero_error_fraction_not_a_divide_by_zero():
    # "No data" is a rate failure, gated elsewhere; it must not masquerade as a parser fault.
    assert StreamStats().parse_error_fraction == 0.0


def test_sequence_gap_is_counted_with_the_number_of_missed_records():
    stats = StreamStats()
    stats.note_record(parse_line(imu_line(10, 100)))
    stats.note_record(parse_line(imu_line(14, 200)))  # 11, 12, 13 lost
    assert stats.sequence_gaps == 1
    assert stats.records_missed == 3
    assert stats.sequence_discontinuity_fraction == pytest.approx(3 / 5)


def test_contiguous_sequence_reports_no_discontinuity():
    stats = StreamStats()
    for line in imu_stream(20):
        stats.note_record(parse_line(line))
    assert stats.sequence_gaps == 0
    assert stats.sequence_discontinuity_fraction == 0.0


def test_sequence_wrap_at_uint32_is_not_a_gap():
    stats = StreamStats()
    stats.note_record(parse_line(imu_line(UINT32_MODULUS - 1, 100)))
    stats.note_record(parse_line(imu_line(0, 200)))
    assert stats.sequence_gaps == 0
    assert stats.records_missed == 0
    assert stats.stream_restarts == 0


def test_duplicate_sequence_is_counted_separately_from_a_gap():
    stats = StreamStats()
    stats.note_record(parse_line(imu_line(5, 100)))
    stats.note_record(parse_line(imu_line(5, 120)))
    assert stats.duplicate_sequences == 1
    assert stats.sequence_gaps == 0


def test_nano_restart_is_not_reported_as_billions_of_missed_records():
    stats = StreamStats()
    stats.note_record(parse_line(imu_line(5_000_000, 100)))
    stats.note_record(parse_line(imu_line(1, 200)))  # Nano rebooted, seq restarted
    assert stats.stream_restarts == 1
    assert stats.records_missed == 0


def test_micros_rollover_is_unwrapped_into_a_monotonic_uptime():
    stats = StreamStats()
    stats.note_record(parse_line(imu_line(1, UINT32_MODULUS - 1000)))
    stats.note_record(parse_line(imu_line(2, 1000)))  # wrapped past 2^32
    assert stats.micros_rollovers == 1
    # 2000 us of true elapsed time across the wrap, not a ~4.29e9 backwards jump.
    assert stats.nano_uptime_us == UINT32_MODULUS - 1000 + 2000


def test_micros_uptime_accumulates_normally_without_a_rollover():
    stats = StreamStats()
    for line in imu_stream(5, start_t_us=1000):
        stats.note_record(parse_line(line))
    assert stats.micros_rollovers == 0
    assert stats.nano_uptime_us == 1000 + 4 * 20000


def test_to_dict_exposes_every_gated_counter():
    stats = StreamStats()
    stats.note_record(parse_line(imu_line(1, 100)))
    payload = stats.to_dict()
    for key in ('parse_error_fraction', 'sequence_discontinuity_fraction', 'records_ok',
                 'crc_errors', 'malformed_errors', 'nano_uptime_us', 'type_counts'):
        assert key in payload
