#!/usr/bin/env python3

import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

import serial


class DiffDriveBase(Node):
    def __init__(self) -> None:
        super().__init__('diff_drive_base')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('wheel_separation', 0.298)
        self.declare_parameter('encoder_ticks_per_rev', 2000.0) # 1974.7, 1779.0
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

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.serial_lock = threading.Lock()
        self.ser: Optional[serial.Serial] = None

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

        self.connect_serial()

        if self.reset_encoders_on_start:
            self.reset_encoders()

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            f'Connected to Arduino bridge on {self.port} @ {self.baudrate} baud'
        )

    def connect_serial(self) -> None:
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
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

    def reset_encoders(self) -> None:
        self.flush_input()
        self.write_command('r')
        # Drain replies briefly; expect "OK"
        for _ in range(5):
            line = self.read_line()
            if line is None:
                continue
            if line == 'OK':
                self.get_logger().info('Arduino encoder reset acknowledged')
                return

    def cmd_callback(self, msg: Twist) -> None:
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        left_rad_s = (v - 0.5 * self.wheel_separation * w) / self.wheel_radius
        right_rad_s = (v + 0.5 * self.wheel_separation * w) / self.wheel_radius

        self.target_left_rad_s = left_rad_s
        self.target_right_rad_s = right_rad_s
        self.last_cmd_time = self.get_clock().now()

    def rad_s_to_counts_per_loop(self, rad_s: float, sign: float) -> int:
        ticks_per_sec = (rad_s / (2.0 * math.pi)) * self.encoder_ticks_per_rev
        ticks_per_loop = ticks_per_sec / self.pid_rate_hz
        return int(round(sign * ticks_per_loop))

    def send_motor_command(self, left_rad_s: float, right_rad_s: float) -> None:
        left_cpl = self.rad_s_to_counts_per_loop(left_rad_s, self.left_motor_sign)
        right_cpl = self.rad_s_to_counts_per_loop(right_rad_s, self.right_motor_sign)
        self.write_command(f'm {left_cpl} {right_cpl}')

    def read_encoders(self) -> Optional[Tuple[int, int]]:
        # Ask for encoder counts
        self.write_command('e')

        # Read several lines if needed, skipping "OK" replies from previous m commands
        for _ in range(10):
            line = self.read_line()
            if line is None or line == '':
                continue

            if line == 'OK':
                continue

            parts = line.split()
            if len(parts) != 2:
                self.get_logger().warning(f'Unexpected encoder reply: "{line}"')
                continue

            try:
                left = int(parts[0])
                right = int(parts[1])
                return left, right
            except ValueError:
                self.get_logger().warning(f'Unexpected encoder reply: "{line}"')
                continue

        return None

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

    def publish_tf(self, stamp) -> None:
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

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout_sec:
            left_cmd = 0.0
            right_cmd = 0.0
        else:
            left_cmd = self.target_left_rad_s
            right_cmd = self.target_right_rad_s

        # Firmware replies "OK" to m
        self.send_motor_command(left_cmd, right_cmd)

        enc = self.read_encoders()
        if enc is None:
            return

        raw_left_ticks, raw_right_ticks = enc
        left_ticks = int(round(self.left_encoder_sign * raw_left_ticks))
        right_ticks = int(round(self.right_encoder_sign * raw_right_ticks))

        if self.prev_left_ticks_total is None or self.prev_right_ticks_total is None:
            self.prev_left_ticks_total = left_ticks
            self.prev_right_ticks_total = right_ticks
            self.prev_time = now
            self.publish_joint_states(now.to_msg())
            self.publish_odom(now.to_msg(), 0.0, 0.0)
            self.publish_tf(now.to_msg())
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
        self.publish_tf(stamp)

        self.prev_left_ticks_total = left_ticks
        self.prev_right_ticks_total = right_ticks
        self.prev_time = now

    def stop_robot(self) -> None:
        try:
            self.write_command('m 0 0')
        except Exception:
            pass

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
    node = DiffDriveBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()