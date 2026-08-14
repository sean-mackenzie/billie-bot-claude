#!/usr/bin/env python3
"""Owns the Sensor Nano serial link and converts its telemetry into the production ROS
contract: /imu/data, /battery_state, /barometer/*, optional /imu/mag, plus bench raw-ADC and
diagnostics topics.

Threading: a bounded reader thread does the blocking `serial.read()` and pushes complete
lines into a `queue.Queue`; a ROS timer drains that queue. The executor therefore never
blocks on the serial port, and a Nano that stops talking degrades into "no messages" (a rate
failure the analyzers catch) instead of a wedged node.

Timestamping: the ROS stamp is captured **in the reader thread at the moment the line is
received**, not when the drain timer gets around to it, so queue latency cannot distort the
rate/gap metrics that UT-IMU-01 gates on. The Nano's own `micros()` value is carried through
to diagnostics as an uptime figure and is *never* interpreted as ROS or Unix epoch time --
it is a 32-bit counter that rolls over every ~71.6 minutes.

Fail-loud policy: a record that fails CRC or field validation is counted and dropped, never
coerced to zeros. Nothing here invents a measurement the Sensor Nano did not send.
"""

import json
import math
import queue
import sys
import threading
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, MagneticField, Temperature
from std_msgs.msg import Float32

from billiebot_sensor_tests.sensor_nano.battery_metrics import ADC_MAX_COUNT, adc_to_volts
from billiebot_sensor_tests.sensor_nano.imu_metrics import (
    ORIENTATION_CONVENTIONS,
    apply_orientation_convention,
    normalize_quaternions,
)
from billiebot_sensor_tests.sensor_nano.protocol import (
    BarometerRecord,
    BatteryRecord,
    CrcError,
    ImuRecord,
    MagnetometerRecord,
    ProtocolError,
    SerialLineAssembler,
    StatusRecord,
    StreamStats,
    parse_line,
)

#: Magnetometer arrives in microtesla (see the firmware header for why not tesla);
#: sensor_msgs/MagneticField is specified in tesla.
_MICROTESLA_TO_TESLA = 1.0e-6


class SensorNanoBridge(Node):

    def __init__(self):
        super().__init__('sensor_nano_bridge')

        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('barometer_frame_id', 'imu_link')
        self.declare_parameter('battery_frame_id', 'base_link')
        self.declare_parameter('orientation_frame_convention', 'bno055_native')
        self.declare_parameter('battery_divider_ratio', 6.0)
        self.declare_parameter('adc_reference_voltage', 5.0)
        self.declare_parameter('battery_cell_count', 3)
        self.declare_parameter('battery_low_voltage', 10.5)
        self.declare_parameter('battery_critical_voltage', 9.9)
        self.declare_parameter('battery_present_min_voltage', 1.0)
        self.declare_parameter('publish_battery', True)
        self.declare_parameter('publish_magnetometer', True)
        self.declare_parameter('publish_barometer', True)
        self.declare_parameter('startup_settle_sec', 2.5)
        self.declare_parameter('status_wait_sec', 5.0)
        self.declare_parameter('serial_read_timeout_sec', 0.2)
        self.declare_parameter('queue_max_lines', 4000)
        self.declare_parameter('drain_period_sec', 0.005)
        self.declare_parameter('diagnostics_period_sec', 1.0)
        self.declare_parameter('fail_on_missing_device', True)
        self.declare_parameter('stats_export_path', '')
        # Provisional diagonal variances. These are NOT a characterized accuracy claim for
        # this unit -- see BLK-15 in the test plan. They are non-zero because
        # robot_localization treats an all-zero covariance as unknown and substitutes its
        # own tiny default, which destabilizes the filter; publishing a stated, configurable,
        # explicitly-provisional value is more honest than either zeros or invented precision.
        self.declare_parameter('orientation_covariance_diagonal', [0.01, 0.01, 0.05])
        self.declare_parameter('angular_velocity_covariance_diagonal', [0.005, 0.005, 0.005])
        self.declare_parameter('linear_acceleration_covariance_diagonal', [0.1, 0.1, 0.1])

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.barometer_frame_id = str(self.get_parameter('barometer_frame_id').value)
        self.battery_frame_id = str(self.get_parameter('battery_frame_id').value)
        self.orientation_convention = str(
            self.get_parameter('orientation_frame_convention').value
        )
        self.divider_ratio = float(self.get_parameter('battery_divider_ratio').value)
        self.adc_reference_voltage = float(self.get_parameter('adc_reference_voltage').value)
        self.battery_cell_count = int(self.get_parameter('battery_cell_count').value)
        self.battery_low_voltage = float(self.get_parameter('battery_low_voltage').value)
        self.battery_critical_voltage = float(
            self.get_parameter('battery_critical_voltage').value
        )
        self.battery_present_min_voltage = float(
            self.get_parameter('battery_present_min_voltage').value
        )
        self.publish_battery = bool(self.get_parameter('publish_battery').value)
        self.publish_magnetometer = bool(self.get_parameter('publish_magnetometer').value)
        self.publish_barometer = bool(self.get_parameter('publish_barometer').value)
        self.startup_settle_sec = float(self.get_parameter('startup_settle_sec').value)
        self.status_wait_sec = float(self.get_parameter('status_wait_sec').value)
        self.read_timeout_sec = float(self.get_parameter('serial_read_timeout_sec').value)
        self.fail_on_missing_device = bool(self.get_parameter('fail_on_missing_device').value)
        self.stats_export_path = str(self.get_parameter('stats_export_path').value)

        if self.orientation_convention not in ORIENTATION_CONVENTIONS:
            raise ValueError(
                f'orientation_frame_convention {self.orientation_convention!r} is not one of '
                f'{ORIENTATION_CONVENTIONS}'
            )

        self._orientation_cov = self._diagonal_covariance('orientation_covariance_diagonal')
        self._angular_cov = self._diagonal_covariance('angular_velocity_covariance_diagonal')
        self._linear_cov = self._diagonal_covariance('linear_acceleration_covariance_diagonal')

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.adc_pub = self.create_publisher(Float32, '/bench/battery/adc', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, '/barometer/pressure', 10)
        self.temperature_pub = self.create_publisher(Temperature, '/barometer/temperature', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/bench/sensor_nano/diagnostics', 10
        )
        # Only created when the firmware is actually configured to send M records: an
        # advertised-but-silent /imu/mag would imply a magnetometer feed that does not exist.
        self.mag_pub = (
            self.create_publisher(MagneticField, '/imu/mag', 10)
            if self.publish_magnetometer else None
        )

        self.stats = StreamStats()
        self._assembler = SerialLineAssembler()
        self._queue = queue.Queue(maxsize=int(self.get_parameter('queue_max_lines').value))
        self._queue_drops = 0
        self._last_status = None
        # BNO055 calibration levels ride in the I record but have no home in sensor_msgs/Imu,
        # so they are republished through diagnostics. UT-IMU-01 records them as
        # informational metrics (they never gate the result -- test plan section 14.8).
        self._last_calibration = None
        self._status_seen = False
        self._serial = None
        self._stop = threading.Event()
        self._reader_thread = None
        self._read_errors = 0

        self._open_serial()

        self.create_timer(float(self.get_parameter('drain_period_sec').value), self._drain_queue)
        self.create_timer(
            float(self.get_parameter('diagnostics_period_sec').value), self._publish_diagnostics
        )

        self.get_logger().info(
            f'Sensor Nano bridge on {self.port} @ {self.baudrate} baud; '
            f'orientation convention={self.orientation_convention}; '
            f'divider_ratio={self.divider_ratio}; adc_reference={self.adc_reference_voltage} V'
        )

    # -- setup helpers -------------------------------------------------------------------

    def _diagonal_covariance(self, parameter_name: str) -> list:
        """Expand a 3-element diagonal parameter into a row-major 3x3 covariance."""
        values = [float(v) for v in self.get_parameter(parameter_name).value]
        if len(values) != 3:
            raise ValueError(
                f'{parameter_name} must have exactly 3 entries (the diagonal), got {len(values)}'
            )
        covariance = [0.0] * 9
        for i, value in enumerate(values):
            covariance[i * 4] = value
        return covariance

    def _open_serial(self) -> None:
        # Imported lazily so that `--help`, parameter inspection, and unit-test imports do
        # not require pyserial to be installed.
        try:
            import serial as pyserial
        except ImportError as exc:
            self._fail(f'pyserial is not installed ({exc}); install python3-serial')
            return

        if not self.port:
            self._fail('no serial port configured; set the `port` parameter '
                       '(e.g. /dev/serial/by-path/...)')
            return

        try:
            self._serial = pyserial.Serial(
                self.port, self.baudrate, timeout=self.read_timeout_sec
            )
        except Exception as exc:  # pyserial raises SerialException, OSError, ValueError
            self._fail(f'could not open {self.port}: {exc}. Check the cable, that the port '
                       'is not held by a serial monitor (lsof), and dialout membership')
            return

        # Opening the port asserts DTR, which resets the Nano into its bootloader. Anything
        # read during that window is bootloader noise or a truncated first record, so it is
        # discarded rather than counted as a parser error.
        self.get_logger().info(
            f'waiting {self.startup_settle_sec:.1f} s for the Nano auto-reset to settle'
        )
        self._sleep(self.startup_settle_sec)
        try:
            self._serial.reset_input_buffer()
        except Exception as exc:
            self.get_logger().warning(f'could not flush the input buffer: {exc}')
        self._assembler.reset()

        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _sleep(self, seconds: float) -> None:
        """Plain wall-clock sleep. Used only during construction, before any timer runs."""
        self._stop.wait(timeout=max(0.0, seconds))

    def _fail(self, message: str) -> None:
        """Hard-fail when the device is required, otherwise degrade to a loud warning."""
        if self.fail_on_missing_device:
            self.get_logger().fatal(message)
            raise SystemExit(1)
        self.get_logger().error(f'{message} (continuing: fail_on_missing_device is false)')

    # -- serial reader -------------------------------------------------------------------

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                # `read(n)` with a timeout returns whatever arrived, so this wakes at least
                # every read_timeout_sec and the thread always observes the stop event.
                waiting = max(1, self._serial.in_waiting)
                chunk = self._serial.read(waiting)
            except Exception as exc:
                if self._stop.is_set():
                    return
                self._read_errors += 1
                self.get_logger().error(f'serial read failed: {exc}')
                self._stop.wait(timeout=0.5)
                continue

            if not chunk:
                continue

            received_ns = self.get_clock().now().nanoseconds
            for line in self._assembler.feed(chunk):
                try:
                    self._queue.put_nowait((received_ns, line))
                except queue.Full:
                    # Shed the oldest line so a burst cannot stall the reader; the drop is
                    # counted and surfaced in diagnostics rather than hidden.
                    try:
                        self._queue.get_nowait()
                        self._queue.put_nowait((received_ns, line))
                    except (queue.Empty, queue.Full):
                        pass
                    self._queue_drops += 1

    # -- drain / publish -----------------------------------------------------------------

    def _drain_queue(self) -> None:
        while True:
            try:
                received_ns, line = self._queue.get_nowait()
            except queue.Empty:
                return
            self._handle_line(received_ns, line)

    def _handle_line(self, received_ns: int, line: str) -> None:
        try:
            record = parse_line(line)
        except CrcError:
            self.stats.note_crc_error()
            return
        except ProtocolError as exc:
            self.stats.note_malformed()
            # Throttled: a baud mismatch would otherwise emit thousands of identical lines.
            self.get_logger().warning(
                f'rejected malformed record: {exc}', throttle_duration_sec=5.0
            )
            return

        self.stats.note_record(record)

        if isinstance(record, ImuRecord):
            self._publish_imu(received_ns, record)
        elif isinstance(record, BatteryRecord):
            self._publish_battery(received_ns, record)
        elif isinstance(record, BarometerRecord):
            self._publish_barometer(received_ns, record)
        elif isinstance(record, MagnetometerRecord):
            self._publish_magnetometer(received_ns, record)
        elif isinstance(record, StatusRecord):
            self._handle_status(record)

    def _stamp(self, received_ns: int):
        return rclpy.time.Time(nanoseconds=received_ns).to_msg()

    def _publish_imu(self, received_ns: int, record: ImuRecord) -> None:
        self._last_calibration = (
            record.cal_sys, record.cal_gyr, record.cal_acc, record.cal_mag
        )
        quaternion = apply_orientation_convention(
            [list(record.quaternion)], self.orientation_convention
        )
        normalized = normalize_quaternions(quaternion)[0]

        msg = Imu()
        msg.header.stamp = self._stamp(received_ns)
        msg.header.frame_id = self.imu_frame_id
        msg.orientation.w = float(normalized[0])
        msg.orientation.x = float(normalized[1])
        msg.orientation.y = float(normalized[2])
        msg.orientation.z = float(normalized[3])
        msg.orientation_covariance = self._orientation_cov
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            float(v) for v in record.angular_velocity
        )
        msg.angular_velocity_covariance = self._angular_cov
        # Specific force including gravity -- see the firmware header. Never the BNO055
        # gravity-removed VECTOR_LINEARACCEL, which would be double-removed by the EKF.
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            float(v) for v in record.linear_acceleration
        )
        msg.linear_acceleration_covariance = self._linear_cov
        self.imu_pub.publish(msg)

    def _publish_battery(self, received_ns: int, record: BatteryRecord) -> None:
        if not self.publish_battery:
            return
        voltage = float(adc_to_volts(
            record.adc_mean, self.adc_reference_voltage, self.divider_ratio, ADC_MAX_COUNT
        ))

        stamp = self._stamp(received_ns)

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.battery_frame_id
        msg.voltage = voltage
        # Only voltage is measured. Everything below is deliberately NaN/empty/UNKNOWN
        # rather than a plausible-looking invention: this bridge cannot see pack current,
        # charge, capacity, state of charge, or individual 3S cell voltages, and a consumer
        # that trusted a fabricated percentage would be badly misled. (base_bridge.py:331
        # synthesizes cell voltages by dividing the pack voltage by the cell count and maps
        # low charge onto POWER_SUPPLY_HEALTH_COLD; both are BLK-06 and are not repeated here.)
        msg.temperature = math.nan
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = math.nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = voltage >= self.battery_present_min_voltage
        msg.cell_voltage = []
        msg.cell_temperature = []
        msg.location = self.battery_frame_id
        msg.serial_number = ''
        self.battery_pub.publish(msg)

        # Published from the same B record, in the same callback, immediately after the
        # BatteryState above. That ordering is the documented pairing invariant UT-BAT-01
        # relies on: the Nth /bench/battery/adc sample is the raw count behind the Nth
        # /battery_state voltage.
        adc_msg = Float32()
        adc_msg.data = float(record.adc_mean)
        self.adc_pub.publish(adc_msg)

    def _publish_barometer(self, received_ns: int, record: BarometerRecord) -> None:
        if not self.publish_barometer:
            return
        stamp = self._stamp(received_ns)

        pressure = FluidPressure()
        pressure.header.stamp = stamp
        pressure.header.frame_id = self.barometer_frame_id
        pressure.fluid_pressure = float(record.pressure_pa)
        pressure.variance = 0.0  # 0.0 is the sensor_msgs convention for "unknown"
        self.pressure_pub.publish(pressure)

        temperature = Temperature()
        temperature.header.stamp = stamp
        temperature.header.frame_id = self.barometer_frame_id
        temperature.temperature = float(record.temperature_c)
        temperature.variance = 0.0
        self.temperature_pub.publish(temperature)

    def _publish_magnetometer(self, received_ns: int, record: MagnetometerRecord) -> None:
        if self.mag_pub is None:
            return
        msg = MagneticField()
        msg.header.stamp = self._stamp(received_ns)
        msg.header.frame_id = self.imu_frame_id
        msg.magnetic_field.x = float(record.mx) * _MICROTESLA_TO_TESLA
        msg.magnetic_field.y = float(record.my) * _MICROTESLA_TO_TESLA
        msg.magnetic_field.z = float(record.mz) * _MICROTESLA_TO_TESLA
        msg.magnetic_field_covariance = [0.0] * 9
        self.mag_pub.publish(msg)

    def _handle_status(self, record: StatusRecord) -> None:
        previous = self._last_status
        self._last_status = record
        if not self._status_seen:
            self._status_seen = True
            self.get_logger().info(
                f'Sensor Nano status: bno_ok={record.bno_ok} bmp_ok={record.bmp_ok} '
                f'fusion_mode=0x{record.fusion_mode:02X}'
            )
            if not record.bno_ok:
                self.get_logger().error(
                    'BNO055 did not initialize (expected I2C 0x28): check VCC/GND and that '
                    'A5->C (SCL) and A4->D (SDA) are not swapped'
                )
            if not record.bmp_ok:
                self.get_logger().error(
                    'BMP280 did not initialize (expected I2C 0x76); the BNO055 path is '
                    'unaffected'
                )
            return

        if previous is not None and previous.reinits != record.reinits:
            self.get_logger().warning(
                f'Sensor Nano performed a peripheral reinitialization '
                f'(total={record.reinits}, i2c_errors={record.i2c_errors})'
            )

    # -- diagnostics ---------------------------------------------------------------------

    def _publish_diagnostics(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        link = DiagnosticStatus()
        link.name = 'sensor_nano: serial link'
        link.hardware_id = self.port
        stats = self.stats.to_dict()
        link.level = (
            DiagnosticStatus.OK if stats['records_ok'] > 0 else DiagnosticStatus.ERROR
        )
        if stats['parse_error_fraction'] > 0.001:
            link.level = DiagnosticStatus.WARN
        link.message = (
            f"{stats['records_ok']} records, "
            f"{stats['parse_error_count']} parse errors"
        )
        link.values = [
            KeyValue(key=key, value=json.dumps(value) if isinstance(value, dict) else str(value))
            for key, value in stats.items()
        ]
        link.values.append(KeyValue(key='queue_drops', value=str(self._queue_drops)))
        link.values.append(KeyValue(key='serial_read_errors', value=str(self._read_errors)))
        link.values.append(
            KeyValue(key='overlong_line_discards', value=str(self._assembler.overlong_discards))
        )
        msg.status.append(link)

        peripherals = DiagnosticStatus()
        peripherals.name = 'sensor_nano: peripherals'
        peripherals.hardware_id = 'SEN0253'
        if self._last_status is None:
            peripherals.level = DiagnosticStatus.WARN
            peripherals.message = 'no status record received yet'
        else:
            status = self._last_status
            healthy = bool(status.bno_ok and status.bmp_ok)
            peripherals.level = (
                DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
            )
            peripherals.message = f'bno_ok={status.bno_ok} bmp_ok={status.bmp_ok}'
            peripherals.values = [
                KeyValue(key='bno_ok', value=str(status.bno_ok)),
                KeyValue(key='bmp_ok', value=str(status.bmp_ok)),
                KeyValue(key='fusion_mode', value=str(status.fusion_mode)),
                KeyValue(key='i2c_errors', value=str(status.i2c_errors)),
                KeyValue(key='imu_read_errors', value=str(status.imu_read_errors)),
                KeyValue(key='bmp_errors', value=str(status.bmp_errors)),
                KeyValue(key='reinits', value=str(status.reinits)),
                KeyValue(key='imu_records_dropped', value=str(status.imu_records_dropped)),
                KeyValue(key='nano_uptime_us', value=str(stats['nano_uptime_us'])),
            ]
        if self._last_calibration is not None:
            cal_sys, cal_gyr, cal_acc, cal_mag = self._last_calibration
            peripherals.values.extend([
                KeyValue(key='cal_sys', value=str(cal_sys)),
                KeyValue(key='cal_gyr', value=str(cal_gyr)),
                KeyValue(key='cal_acc', value=str(cal_acc)),
                KeyValue(key='cal_mag', value=str(cal_mag)),
            ])
        msg.status.append(peripherals)

        self.diagnostics_pub.publish(msg)

    # -- shutdown ------------------------------------------------------------------------

    def export_stats(self) -> None:
        """Write the parser/peripheral counters to exports/ as a redundant, bag-independent
        record. The bagged diagnostics topic stays authoritative; this is what makes a run
        diagnosable even when the bag is missing or truncated."""
        if not self.stats_export_path:
            return
        payload = dict(self.stats.to_dict())
        payload['queue_drops'] = self._queue_drops
        payload['serial_read_errors'] = self._read_errors
        payload['overlong_line_discards'] = self._assembler.overlong_discards
        payload['port'] = self.port
        payload['baudrate'] = self.baudrate
        payload['orientation_frame_convention'] = self.orientation_convention
        payload['battery_divider_ratio'] = self.divider_ratio
        payload['adc_reference_voltage'] = self.adc_reference_voltage
        if self._last_status is not None:
            payload['last_status'] = {
                'bno_ok': self._last_status.bno_ok,
                'bmp_ok': self._last_status.bmp_ok,
                'fusion_mode': self._last_status.fusion_mode,
                'i2c_errors': self._last_status.i2c_errors,
                'imu_read_errors': self._last_status.imu_read_errors,
                'bmp_errors': self._last_status.bmp_errors,
                'reinits': self._last_status.reinits,
                'imu_records_dropped': self._last_status.imu_records_dropped,
            }
        if self._last_calibration is not None:
            cal_sys, cal_gyr, cal_acc, cal_mag = self._last_calibration
            payload['last_calibration'] = {
                'cal_sys': cal_sys, 'cal_gyr': cal_gyr,
                'cal_acc': cal_acc, 'cal_mag': cal_mag,
            }
        try:
            path = Path(self.stats_export_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2) + '\n')
        except OSError as exc:
            self.get_logger().error(f'could not write {self.stats_export_path}: {exc}')

    def destroy_node(self):
        self._stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)
        self.export_stats()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorNanoBridge()
    except SystemExit as exc:
        rclpy.shutdown()
        return int(exc.code) if exc.code is not None else 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
