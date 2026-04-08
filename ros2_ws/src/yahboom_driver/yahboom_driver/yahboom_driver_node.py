"""
Yahboom YB-ERF01-V3.0 ESC Driver Node
======================================
Subscribes to /cmd_vel (Twist) and sends serial commands to the Yahboom board
to control two ESCs in a differential-drive configuration.

Serial protocol (Yahboom standard):
  Header:   0xFF 0xFE
  Cmd ID:   0x01 (set motor speeds)
  Length:    0x06 (6 data bytes)
  Data:     left_speed_H, left_speed_L, left_dir,
            right_speed_H, right_speed_L, right_dir
  Checksum: sum(all bytes after header) & 0xFF

  Direction: 0x01 = forward, 0x00 = backward
  Speed range: 0-1000 (mapped from m/s via max_speed parameter)

If your board uses a different protocol, adjust _build_motor_frame().
"""

import serial
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class YahboomDriverNode(Node):

    def __init__(self) -> None:
        super().__init__('yahboom_driver')

        # Serial port parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Robot geometry
        self.declare_parameter('wheel_base', 0.3)  # metres between left/right thrusters

        # Speed limits
        self.declare_parameter('max_speed', 1.0)       # m/s corresponding to ESC full throttle
        self.declare_parameter('esc_max_value', 1000)   # max value the board accepts (0-1000)

        # Safety: auto-stop if no cmd_vel received within this many seconds
        self.declare_parameter('cmd_timeout', 0.5)

        # Update rate for sending serial commands
        self.declare_parameter('update_rate_hz', 20.0)

        # Dry-run mode: log commands without opening serial port
        self.declare_parameter('dry_run', False)

        self._serial = None
        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()

        dry_run = bool(self.get_parameter('dry_run').value)
        if not dry_run:
            self._open_serial()
        else:
            self.get_logger().warn('DRY RUN mode: serial commands will be logged but not sent')

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_callback, 10)

        # Publish actual ESC values for debugging / UI feedback
        self.esc_pub = self.create_publisher(Float32MultiArray, '/esc_values', 10)

        period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.timer = self.create_timer(period, self._send_command)

        port = self.get_parameter('serial_port').value
        self.get_logger().info(
            f'Yahboom driver ready — port={port}, '
            f'wheel_base={self.get_parameter("wheel_base").value}m, '
            f'dry_run={dry_run}'
        )

    def _open_serial(self) -> None:
        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baud_rate').value)
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Serial port opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self._serial = None

    def _cmd_callback(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _send_command(self) -> None:
        # Safety timeout — stop motors if no recent command
        timeout = float(self.get_parameter('cmd_timeout').value)
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > timeout:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self._last_cmd.linear.x
            angular_z = self._last_cmd.angular.z

        # Differential drive kinematics: convert (linear, angular) to (left, right)
        wheel_base = float(self.get_parameter('wheel_base').value)
        left_vel = linear_x - angular_z * (wheel_base / 2.0)
        right_vel = linear_x + angular_z * (wheel_base / 2.0)

        # Map velocities to ESC values (0 to esc_max_value)
        max_speed = float(self.get_parameter('max_speed').value)
        esc_max = int(self.get_parameter('esc_max_value').value)

        left_val = int(min(abs(left_vel) / max_speed, 1.0) * esc_max)
        right_val = int(min(abs(right_vel) / max_speed, 1.0) * esc_max)

        left_dir = 0x01 if left_vel >= 0 else 0x00
        right_dir = 0x01 if right_vel >= 0 else 0x00

        # Build and send the serial frame
        frame = self._build_motor_frame(left_val, left_dir, right_val, right_dir)

        dry_run = bool(self.get_parameter('dry_run').value)
        if dry_run:
            self.get_logger().info(
                f'[DRY] L={left_val:4d} ({"FWD" if left_dir else "REV"})  '
                f'R={right_val:4d} ({"FWD" if right_dir else "REV"})  '
                f'frame={frame.hex(" ")}'
            )
        elif self._serial and self._serial.is_open:
            try:
                self._serial.write(frame)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')

        # Publish ESC values for UI / debugging
        esc_msg = Float32MultiArray()
        signed_left = left_val if left_dir else -left_val
        signed_right = right_val if right_dir else -right_val
        esc_msg.data = [float(signed_left), float(signed_right)]
        self.esc_pub.publish(esc_msg)

    def _build_motor_frame(
        self, left_val: int, left_dir: int, right_val: int, right_dir: int
    ) -> bytes:
        """
        Build the serial frame for the Yahboom board.

        Protocol:
          0xFF 0xFE  <cmd>  <len>  <data...>  <checksum>

        Adjust this method if your board uses a different protocol.
        """
        cmd = 0x01   # motor speed command
        length = 0x06  # 6 data bytes

        left_h = (left_val >> 8) & 0xFF
        left_l = left_val & 0xFF
        right_h = (right_val >> 8) & 0xFF
        right_l = right_val & 0xFF

        data = bytes([cmd, length, left_h, left_l, left_dir, right_h, right_l, right_dir])
        checksum = sum(data) & 0xFF

        return b'\xFF\xFE' + data + bytes([checksum])

    def destroy_node(self) -> None:
        # Send stop command before shutting down
        if self._serial and self._serial.is_open:
            stop_frame = self._build_motor_frame(0, 0x01, 0, 0x01)
            try:
                self._serial.write(stop_frame)
            except serial.SerialException:
                pass
            self._serial.close()
            self.get_logger().info('Serial port closed, motors stopped.')
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = YahboomDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
