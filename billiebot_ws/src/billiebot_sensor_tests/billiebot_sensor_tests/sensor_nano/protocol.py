"""Sensor Nano serial protocol: CRC-16/CCITT-FALSE framing, strict record parsing, and
stream integrity tracking.

Pure Python -- no rclpy, no pyserial, no numpy -- so the wire format is unit-testable on any
host without a ROS install, and so a protocol bug can never hide behind a hardware failure.

Wire format (firmware/sensor_nano/sensor_nano.ino emits exactly this):

    <TYPE>,<field>,<field>,...*<CRC>\\n

`<CRC>` is four uppercase hex digits of CRC-16/CCITT-FALSE over the payload bytes preceding
the `*` (the type character and its commas included, the `*` and CRC excluded).

Record types:

    I  IMU        seq,t_us,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,cal_sys,cal_gyr,cal_acc,cal_mag
    B  battery    seq,t_us,adc_mean
    P  barometer  seq,t_us,pressure_pa,temperature_c
    M  magnetom.  seq,t_us,mx,my,mz
    S  status     seq,t_us,bno_ok,bmp_ok,fusion_mode,i2c_errors,imu_read_errors,
                  bmp_errors,reinits,imu_records_dropped

Units are SI at the wire, converted in firmware: quaternion dimensionless, gyro rad/s,
acceleration m/s^2 *including gravity* (BNO055 VECTOR_ACCELEROMETER, never VECTOR_LINEARACCEL
-- see the firmware README), magnetometer tesla, pressure Pa, temperature degrees C. The
battery field is a raw averaged 10-bit ADC count and is deliberately NOT volts: the divider
ratio and ADC reference live on the Jetson (sensor_nano_bridge), never in firmware.

Parsing is strict by design. A record with the wrong field count, an unparseable numeric
field, a non-finite float, or a bad CRC raises -- it is never coerced to zeros, because a
silently-zeroed quaternion or ADC count would look like plausible sensor data downstream.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

#: CRC-16/CCITT-FALSE parameters. Mirrored bit-for-bit by crc16_update() in the firmware.
CRC16_POLY = 0x1021
CRC16_INIT = 0xFFFF

#: Canonical CRC-16/CCITT-FALSE check value: crc16_ccitt_false(b'123456789') == 0x29B1.
#: Asserted by both the Python test suite and the firmware's boot-time self-check, so the
#: two implementations can never silently diverge.
CRC16_CHECK_INPUT = b'123456789'
CRC16_CHECK_VALUE = 0x29B1

#: Sequence numbers and micros() timestamps are both uint32 on the ATmega328P.
UINT32_MODULUS = 1 << 32

#: A forward sequence jump larger than this is read as a Nano restart (or a reconnect to a
#: Nano that has been running a while), not as that many genuinely dropped records. At the
#: ~58 records/s this firmware emits, 1e6 records is roughly 4.8 hours of continuous loss --
#: far beyond any plausible serial dropout, and exactly what a restart looks like modulo 2^32.
SEQ_RESTART_THRESHOLD = 1_000_000

RECORD_TYPES = ('I', 'B', 'P', 'M', 'S')


class ProtocolError(ValueError):
    """Base class for every rejected line. Carries the offending text for the log."""

    def __init__(self, message: str, line: str = ''):
        super().__init__(message)
        self.line = line


class CrcError(ProtocolError):
    """Framing was intact but the checksum did not match -- corrupted in transit."""


class MalformedRecordError(ProtocolError):
    """Unknown type, wrong field count, unparseable number, or a non-finite value."""


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no input/output reflection, no final XOR.

    Byte-at-a-time so it reads the same as the firmware's crc16_update() helper.
    """
    crc = CRC16_INIT
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC16_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_line(record_type: str, fields: Sequence) -> str:
    """Assemble a payload plus its CRC suffix, the inverse of parse_line().

    Used by the test fixtures and by anyone hand-crafting a stimulus stream; the firmware
    builds its lines in C but over byte-identical payload text.
    """
    payload = ','.join([record_type] + [str(f) for f in fields])
    crc = crc16_ccitt_false(payload.encode('ascii'))
    return f'{payload}*{crc:04X}'


@dataclass(frozen=True)
class Record:
    """Fields common to every record type."""

    seq: int
    t_us: int


@dataclass(frozen=True)
class ImuRecord(Record):
    qw: float
    qx: float
    qy: float
    qz: float
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float
    cal_sys: int
    cal_gyr: int
    cal_acc: int
    cal_mag: int

    @property
    def quaternion(self) -> tuple:
        """(w, x, y, z), the BNO055 fusion quaternion in its native world frame.

        No axis remap is applied anywhere in firmware or parser -- see
        sensor_nano_bridge's `orientation_frame_convention` parameter for the
        documented, opt-in conversion.
        """
        return (self.qw, self.qx, self.qy, self.qz)

    @property
    def angular_velocity(self) -> tuple:
        """(x, y, z) in rad/s."""
        return (self.gx, self.gy, self.gz)

    @property
    def linear_acceleration(self) -> tuple:
        """(x, y, z) in m/s^2, specific force *including* gravity."""
        return (self.ax, self.ay, self.az)


@dataclass(frozen=True)
class BatteryRecord(Record):
    #: Mean of the A0 samples accumulated since the previous battery record. Fractional
    #: because it is an average of 10-bit counts, so it can exceed 10-bit resolution.
    adc_mean: float


@dataclass(frozen=True)
class BarometerRecord(Record):
    pressure_pa: float
    temperature_c: float


@dataclass(frozen=True)
class MagnetometerRecord(Record):
    mx: float
    my: float
    mz: float

    @property
    def magnetic_field(self) -> tuple:
        """(x, y, z) in tesla."""
        return (self.mx, self.my, self.mz)


@dataclass(frozen=True)
class StatusRecord(Record):
    bno_ok: int
    bmp_ok: int
    fusion_mode: int
    i2c_errors: int
    imu_read_errors: int
    bmp_errors: int
    reinits: int
    imu_records_dropped: int


#: type -> (dataclass, [(field_name, converter), ...]) after the leading seq,t_us pair.
_SCHEMA = {
    'I': (ImuRecord, [
        ('qw', float), ('qx', float), ('qy', float), ('qz', float),
        ('gx', float), ('gy', float), ('gz', float),
        ('ax', float), ('ay', float), ('az', float),
        ('cal_sys', int), ('cal_gyr', int), ('cal_acc', int), ('cal_mag', int),
    ]),
    'B': (BatteryRecord, [('adc_mean', float)]),
    'P': (BarometerRecord, [('pressure_pa', float), ('temperature_c', float)]),
    'M': (MagnetometerRecord, [('mx', float), ('my', float), ('mz', float)]),
    'S': (StatusRecord, [
        ('bno_ok', int), ('bmp_ok', int), ('fusion_mode', int),
        ('i2c_errors', int), ('imu_read_errors', int), ('bmp_errors', int),
        ('reinits', int), ('imu_records_dropped', int),
    ]),
}


def _parse_uint32(text: str, name: str, line: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise MalformedRecordError(f'{name} is not an integer: {text!r}', line) from None
    if not 0 <= value < UINT32_MODULUS:
        raise MalformedRecordError(f'{name} out of uint32 range: {value}', line)
    return value


def parse_line(line: str) -> Record:
    """Parse one complete record. Raises CrcError or MalformedRecordError on anything else.

    `line` must already be a single line with its terminator stripped; partial lines are the
    reader's problem, not the parser's (see SerialLineAssembler).
    """
    text = line.strip()
    if not text:
        raise MalformedRecordError('empty line', line)

    payload, sep, crc_text = text.rpartition('*')
    if not sep:
        raise MalformedRecordError('no CRC delimiter', line)
    if len(crc_text) != 4:
        raise MalformedRecordError(f'CRC field is not 4 hex digits: {crc_text!r}', line)
    try:
        expected_crc = int(crc_text, 16)
    except ValueError:
        raise MalformedRecordError(f'CRC field is not hex: {crc_text!r}', line) from None

    try:
        payload_bytes = payload.encode('ascii')
    except UnicodeEncodeError:
        raise MalformedRecordError('payload contains non-ASCII bytes', line) from None

    actual_crc = crc16_ccitt_false(payload_bytes)
    if actual_crc != expected_crc:
        raise CrcError(f'CRC mismatch: got {actual_crc:04X}, expected {expected_crc:04X}', line)

    parts = payload.split(',')
    record_type = parts[0]
    if record_type not in _SCHEMA:
        raise MalformedRecordError(f'unknown record type {record_type!r}', line)

    cls, spec = _SCHEMA[record_type]
    expected_len = 1 + 2 + len(spec)  # type + seq + t_us + payload fields
    if len(parts) != expected_len:
        raise MalformedRecordError(
            f'{record_type} record has {len(parts)} fields, expected {expected_len}', line
        )

    seq = _parse_uint32(parts[1], 'seq', line)
    t_us = _parse_uint32(parts[2], 't_us', line)

    values = {}
    for (name, converter), raw in zip(spec, parts[3:]):
        try:
            value = converter(raw)
        except ValueError:
            raise MalformedRecordError(
                f'{record_type}.{name} is not a {converter.__name__}: {raw!r}', line
            ) from None
        # A NaN/inf on the wire means the firmware produced garbage. Surfacing it as a
        # parse failure is the whole point -- publishing it would poison /imu/data and the
        # EKF far downstream of the actual fault.
        if converter is float and not math.isfinite(value):
            raise MalformedRecordError(f'{record_type}.{name} is not finite: {raw!r}', line)
        values[name] = value

    return cls(seq=seq, t_us=t_us, **values)


class SerialLineAssembler:
    """Turns arbitrary byte chunks from a serial port into complete lines.

    Holds a partial trailing line across reads so a record split across two `read()` calls is
    reassembled rather than being counted as two malformed records. `max_line_length` bounds
    the buffer so a Nano stuck emitting a stream with no newline cannot grow it without limit.
    """

    def __init__(self, max_line_length: int = 512):
        self.max_line_length = max_line_length
        self._buffer = bytearray()
        self.overlong_discards = 0

    def feed(self, chunk: bytes) -> list:
        """Append `chunk` and return every complete line it produced, as stripped `str`."""
        lines = []
        self._buffer.extend(chunk)
        while True:
            index = self._buffer.find(b'\n')
            if index < 0:
                break
            raw = bytes(self._buffer[:index])
            del self._buffer[:index + 1]
            text = raw.decode('ascii', errors='replace').strip()
            if text:
                lines.append(text)
        if len(self._buffer) > self.max_line_length:
            # No newline in a buffer this large means we are mid-garbage (line noise, a
            # half-flashed Nano, or a baud mismatch). Drop it and resynchronise on the next
            # newline rather than accumulating forever.
            self._buffer.clear()
            self.overlong_discards += 1
        return lines

    def reset(self) -> None:
        self._buffer.clear()


class StreamStats:
    """Running integrity counters for one Sensor Nano serial session.

    Tracks CRC/malformed fractions, per-type counts, sequence continuity across uint32
    rollover, and an unwrapped micros() clock. All of it feeds
    /bench/sensor_nano/diagnostics and the UT-IMU-01 / UT-BAT-01 parser-error gates.
    """

    def __init__(self, seq_restart_threshold: int = SEQ_RESTART_THRESHOLD):
        self.seq_restart_threshold = seq_restart_threshold
        self.lines_total = 0
        self.records_ok = 0
        self.crc_errors = 0
        self.malformed_errors = 0
        self.type_counts = {t: 0 for t in RECORD_TYPES}
        self.sequence_gaps = 0
        self.records_missed = 0
        self.duplicate_sequences = 0
        self.stream_restarts = 0
        self._last_seq: Optional[int] = None
        self._last_t_us: Optional[int] = None
        self._t_us_unwrapped = 0
        self.micros_rollovers = 0

    # -- counters ------------------------------------------------------------------------

    def note_crc_error(self) -> None:
        self.lines_total += 1
        self.crc_errors += 1

    def note_malformed(self) -> None:
        self.lines_total += 1
        self.malformed_errors += 1

    def note_record(self, record: Record) -> None:
        """Account for one successfully parsed record and advance the seq/time trackers."""
        self.lines_total += 1
        self.records_ok += 1
        type_key = _type_key(record)
        if type_key in self.type_counts:
            self.type_counts[type_key] += 1
        self._track_sequence(record.seq)
        self._track_micros(record.t_us)

    def _track_sequence(self, seq: int) -> None:
        if self._last_seq is None:
            self._last_seq = seq
            return
        delta = (seq - self._last_seq) % UINT32_MODULUS
        if delta == 0:
            self.duplicate_sequences += 1
        elif delta > self.seq_restart_threshold:
            # Either the Nano rebooted (seq restarts near 0) or we attached mid-stream.
            # Counting this as ~4 billion dropped records would be nonsense.
            self.stream_restarts += 1
        elif delta > 1:
            self.sequence_gaps += 1
            self.records_missed += delta - 1
        self._last_seq = seq

    def _track_micros(self, t_us: int) -> None:
        if self._last_t_us is None:
            self._last_t_us = t_us
            self._t_us_unwrapped = t_us
            return
        delta = (t_us - self._last_t_us) % UINT32_MODULUS
        if t_us < self._last_t_us:
            self.micros_rollovers += 1
        self._t_us_unwrapped += delta
        self._last_t_us = t_us

    # -- derived -------------------------------------------------------------------------

    @property
    def parse_error_count(self) -> int:
        return self.crc_errors + self.malformed_errors

    @property
    def parse_error_fraction(self) -> float:
        """Fraction of received lines rejected. 0.0 when nothing has been received yet --
        an empty stream is a rate failure, not a parser failure, and is gated elsewhere."""
        if self.lines_total == 0:
            return 0.0
        return self.parse_error_count / self.lines_total

    @property
    def sequence_discontinuity_fraction(self) -> float:
        """Missed records as a fraction of records that *should* have arrived."""
        expected = self.records_ok + self.records_missed
        if expected == 0:
            return 0.0
        return self.records_missed / expected

    @property
    def nano_uptime_us(self) -> int:
        """micros() unwrapped across 2^32 rollover. Diagnostic only -- this is the Nano's
        own clock and is never treated as ROS/Unix epoch time."""
        return self._t_us_unwrapped

    def to_dict(self) -> dict:
        return {
            'lines_total': self.lines_total,
            'records_ok': self.records_ok,
            'crc_errors': self.crc_errors,
            'malformed_errors': self.malformed_errors,
            'parse_error_count': self.parse_error_count,
            'parse_error_fraction': self.parse_error_fraction,
            'type_counts': dict(self.type_counts),
            'sequence_gaps': self.sequence_gaps,
            'records_missed': self.records_missed,
            'sequence_discontinuity_fraction': self.sequence_discontinuity_fraction,
            'duplicate_sequences': self.duplicate_sequences,
            'stream_restarts': self.stream_restarts,
            'micros_rollovers': self.micros_rollovers,
            'nano_uptime_us': self.nano_uptime_us,
        }


def _type_key(record: Record) -> str:
    for key, (cls, _spec) in _SCHEMA.items():
        if isinstance(record, cls):
            return key
    return ''
