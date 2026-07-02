#!/usr/bin/env python3
"""BillieBot base bridge — differential drive interface.

Adapted from reference diff_drive_base.py with additions:
- MockSerial for hardware-free testing (mock:=true)
- Battery voltage reading via Arduino 'a' command
- E-stop ROS service
- Configurable TF publishing (disable when EKF is active)
- IMU path guarded behind use_imu parameter
"""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, BatteryState
from tf2_ros import TransformBroadcaster

from billiebot_interfaces.srv import EStop


class MockSerial:
    """Simulates Arduino serial responses for hardware-free testing."""

    def __init__(self, node: Node):
        self._node = node
        self._left_ticks = 0
        self._right_ticks = 0
        self._left_speed_cpl = 0  # counts per loop
        self._right_speed_cpl = 0
        self._lock = threading.Lock()
        self._response_queue: list[str] = []
        self._last_update = time.monotonic()

    def write(self, data: bytes) -> None:
        cmd = data.decode('utf-8', errors='ignore').strip()
        with self._lock:
            if cmd == 'r':
                self._left_ticks = 0
                self._right_ticks = 0
                self._response_queue.append('OK')
            elif cmd == 'e':
                # Simulate encoder ticks accumulated since last read
                now = time.monotonic()
                dt = now - self._last_update
                self._last_update = now
                # PID rate is 30 Hz, so counts_per_loop * rate * dt = ticks
                rate = 30.0
                self._left_ticks += int(self._left_speed_cpl * rate * dt)
                self._right_ticks += int(self._right_speed_cpl * rate * dt)
                self._response_queue.append(
                    f'{self._left_ticks} {self._right_ticks}'
                )
            elif cmd.startswith('m '):
                parts = cmd.split()
                if len(parts) == 3:
                    self._left_speed_cpl = int(parts[1])
                    self._right_speed_cpl = int(parts[2])
                self._response_queue.append('OK')
            elif cmd.startswith('a '):
                # Simulate battery ADC reading: ~12.6V through 1/6 divider
                # ADC = V_batt / 6.0 * 1023 / 5.0
                adc_val = int(12.6 / 6.0 * 1023.0 / 5.0)
                self._response_queue.append(str(adc_val))
            else:
                self._response_queue.append('OK')

    def readline(self) -> bytes:
        with self._lock:
            if self._response_queue:
                line = self._response_queue.pop(0)
                return (line + '\r\n').encode('utf-8')
        return b''

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._response_queue.clear()

    def reset_output_buffer(self) -> None:
        pass

    def close(self) -> None:
        pass


class BaseBridge(Node):
    def __init__(self) -> None:
        super().__init__('base_bridge')

        # Declare parameters (reference + extensions)
        self.declare_parameter('mock', False)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('wheel_separation', 0.298)
        self.declare_parameter('encoder_ticks_per_rev', 2000.0)
        self.declare_parameter('pid_rate_hz', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')
        self.declare_parameter('left_motor_sign', 1.0)
        self.declare_parameter('right_motor_sign', 1.0)
        self.declare_parameter('left_encoder_sign', 1.0)
        self.declare_parameter('right_encoder_sign', 1.0)
        self.declare_parameter('reset_encoders_on_start', True)
        # New parameters
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_imu', False)
        self.declare_parameter('battery_pin', 0)
        self.declare_parameter('battery_divider_ratio', 6.0)
        self.declare_parameter('battery_cell_count', 3)
        self.declare_parameter('battery_read_interval_sec', 1.0)
        self.declare_parameter('battery_low_voltage', 10.5)    # 3.5V/cell * 3
        self.declare_parameter('battery_critical_voltage', 9.9)  # 3.3V/cell * 3

        # Read parameters
        self.mock = bool(self.get_parameter('mock').value)
        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.encoder_ticks_per_rev = float(self.get_parameter('encoder_ticks_per_rev').value)
        self.pid_rate_hz = float(self.get_parameter('pid_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.left_joint_name = str(self.get_parameter('left_joint_name').value)
        self.right_joint_name = str(self.get_parameter('right_joint_name').value)
        self.left_motor_sign = float(self.get_parameter('left_motor_sign').value)
        self.right_motor_sign = float(self.get_parameter('right_motor_sign').value)
        self.left_encoder_sign = float(self.get_parameter('left_encoder_sign').value)
        self.right_encoder_sign = float(self.get_parameter('right_encoder_sign').value)
        self.reset_encoders_on_start = bool(self.get_parameter('reset_encoders_on_start').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.battery_pin = int(self.get_parameter('battery_pin').value)
        self.battery_divider_ratio = float(self.get_parameter('battery_divider_ratio').value)
        self.battery_cell_count = int(self.get_parameter('battery_cell_count').value)
        self.battery_read_interval = float(self.get_parameter('battery_read_interval_sec').value)
        self.battery_low_voltage = float(self.get_parameter('battery_low_voltage').value)
        self.battery_critical_voltage = float(self.get_parameter('battery_critical_voltage').value)

        # Publishers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # E-stop service
        self._estopped = False
        self.estop_srv = self.create_service(EStop, '/e_stop', self.estop_callback)

        # Serial
        self.serial_lock = threading.Lock()
        self.ser = None
        self.connect_serial()

        # Drive state
        self.last_cmd_time = self.get_clock().now()
        self.target_left_rad_s = 0.0
        self.target_right_rad_s = 0.0

        self.prev_left_ticks_total: Optional[int] = None
        self.prev_right_ticks_total: Optional[int] = None

        self.left_pos = 0.0
        self.right_pos = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_time = self.get_clock().now()

        if self.reset_encoders_on_start:
            self.reset_encoders()

        # Main update timer
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.update)

        # Battery reading timer
        self._last_battery_time = self.get_clock().now()
        self.battery_timer = self.create_timer(
            self.battery_read_interval, self.read_battery
        )

        mode_str = 'MOCK' if self.mock else f'{self.port} @ {self.baudrate}'
        self.get_logger().info(f'BaseBridge started ({mode_str})')

    # -- Serial --

    def connect_serial(self) -> None:
        if self.mock:
            self.ser = MockSerial(self)
            return
        import serial as pyserial
        self.ser = pyserial.Serial(self.port, self.baudrate, timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def write_command(self, cmd: str) -> None:
        if self.ser is None:
            return
        with self.serial_lock:
            self.ser.write((cmd + '\r').encode('utf-8'))

    def read_line(self) -> Optional[str]:
        if self.ser is None:
            return None
        with self.serial_lock:
            raw = self.ser.readline()
        if not raw:
            return None
        try:
            return raw.decode('utf-8', errors='ignore').strip()
        except Exception:
            return None

    def flush_input(self) -> None:
        if self.ser is None:
            return
        with self.serial_lock:
            self.ser.reset_input_buffer()

    # -- Encoders --

    def reset_encoders(self) -> None:
        self.flush_input()
        self.write_command('r')
        for _ in range(5):
            line = self.read_line()
            if line and line == 'OK':
                self.get_logger().info('Encoder reset acknowledged')
                return

    def read_encoders(self) -> Optional[Tuple[int, int]]:
        self.write_command('e')
        for _ in range(10):
            line = self.read_line()
            if line is None or line == '' or line == 'OK':
                continue
            parts = line.split()
            if len(parts) != 2:
                continue
            try:
                return int(parts[0]), int(parts[1])
            except ValueError:
                continue
        return None

    # -- Motor commands --

    def cmd_callback(self, msg: Twist) -> None:
        if self._estopped:
            return
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        self.target_left_rad_s = (v - 0.5 * self.wheel_separation * w) / self.wheel_radius
        self.target_right_rad_s = (v + 0.5 * self.wheel_separation * w) / self.wheel_radius
        self.last_cmd_time = self.get_clock().now()

    def rad_s_to_counts_per_loop(self, rad_s: float, sign: float) -> int:
        ticks_per_sec = (rad_s / (2.0 * math.pi)) * self.encoder_ticks_per_rev
        ticks_per_loop = ticks_per_sec / self.pid_rate_hz
        return int(round(sign * ticks_per_loop))

    def send_motor_command(self, left_rad_s: float, right_rad_s: float) -> None:
        left_cpl = self.rad_s_to_counts_per_loop(left_rad_s, self.left_motor_sign)
        right_cpl = self.rad_s_to_counts_per_loop(right_rad_s, self.right_motor_sign)
        self.write_command(f'm {left_cpl} {right_cpl}')

    def stop_robot(self) -> None:
        try:
            self.write_command('m 0 0')
        except Exception:
            pass

    # -- E-stop service --

    def estop_callback(self, request, response):
        if request.engage:
            self._estopped = True
            self.target_left_rad_s = 0.0
            self.target_right_rad_s = 0.0
            self.stop_robot()
            response.success = True
            response.message = 'E-stop engaged — motors stopped'
            self.get_logger().warn('E-STOP ENGAGED')
        else:
            self._estopped = False
            response.success = True
            response.message = 'E-stop released'
            self.get_logger().info('E-stop released')
        return response

    # -- Battery --

    def read_battery(self) -> None:
        self.write_command(f'a {self.battery_pin}')
        for _ in range(5):
            line = self.read_line()
            if line is None or line == '' or line == 'OK':
                continue
            try:
                adc_val = int(line)
                voltage = adc_val * 5.0 / 1023.0 * self.battery_divider_ratio
                self.publish_battery(voltage)
                return
            except ValueError:
                continue

    def publish_battery(self, voltage: float) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.cell_voltage = [voltage / self.battery_cell_count] * self.battery_cell_count
        msg.present = True

        if voltage <= self.battery_critical_voltage:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif voltage <= self.battery_low_voltage:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_COLD
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        self.battery_pub.publish(msg)

    # -- Odometry publishing --

    def publish_joint_states(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [self.left_joint_name, self.right_joint_name]
        msg.position = [self.left_pos, self.right_pos]
        msg.velocity = [self.left_vel, self.right_vel]
        self.joint_pub.publish(msg)

    def publish_odom(self, stamp, vx: float, wz: float) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz
        self.odom_pub.publish(msg)

    def publish_tf_odom(self, stamp) -> None:
        if not self._publish_tf:
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    # -- Main update loop --

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Command timeout
        if self._estopped or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout_sec:
            left_cmd = 0.0
            right_cmd = 0.0
        else:
            left_cmd = self.target_left_rad_s
            right_cmd = self.target_right_rad_s

        self.send_motor_command(left_cmd, right_cmd)

        enc = self.read_encoders()
        if enc is None:
            return

        raw_left, raw_right = enc
        left_ticks = int(round(self.left_encoder_sign * raw_left))
        right_ticks = int(round(self.right_encoder_sign * raw_right))

        if self.prev_left_ticks_total is None or self.prev_right_ticks_total is None:
            self.prev_left_ticks_total = left_ticks
            self.prev_right_ticks_total = right_ticks
            self.prev_time = now
            stamp = now.to_msg()
            self.publish_joint_states(stamp)
            self.publish_odom(stamp, 0.0, 0.0)
            self.publish_tf_odom(stamp)
            return

        dleft_ticks = left_ticks - self.prev_left_ticks_total
        dright_ticks = right_ticks - self.prev_right_ticks_total

        rad_per_tick = (2.0 * math.pi) / self.encoder_ticks_per_rev
        dleft_rad = dleft_ticks * rad_per_tick
        dright_rad = dright_ticks * rad_per_tick

        self.left_pos += dleft_rad
        self.right_pos += dright_rad
        self.left_vel = dleft_rad / dt
        self.right_vel = dright_rad / dt

        dl = dleft_rad * self.wheel_radius
        dr = dright_rad * self.wheel_radius
        dc = 0.5 * (dl + dr)
        dyaw = (dr - dl) / self.wheel_separation

        if abs(dyaw) < 1e-12:
            self.x += dc * math.cos(self.yaw)
            self.y += dc * math.sin(self.yaw)
        else:
            self.x += dc * math.cos(self.yaw + 0.5 * dyaw)
            self.y += dc * math.sin(self.yaw + 0.5 * dyaw)

        self.yaw += dyaw
        vx = dc / dt
        wz = dyaw / dt

        stamp = now.to_msg()
        self.publish_joint_states(stamp)
        self.publish_odom(stamp, vx, wz)
        self.publish_tf_odom(stamp)

        self.prev_left_ticks_total = left_ticks
        self.prev_right_ticks_total = right_ticks
        self.prev_time = now

    # -- Shutdown --

    def destroy_node(self):
        self.stop_robot()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
