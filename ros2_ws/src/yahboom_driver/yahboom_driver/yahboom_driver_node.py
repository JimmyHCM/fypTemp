"""
Yahboom YB-ERF01-V3.0 ESC Driver Node
======================================
Subscribes to /cmd_vel (Twist) and sends serial commands to the Yahboom
Rosmaster board to control two bidirectional ESCs on PWM servo ports S1 & S2
in a differential-drive configuration.

Serial protocol (Rosmaster V3.3.x):
  Header:     0xFF 0xFC
  Length:      total frame length - 1 (stored at byte index 2)
  Function:   0x03 = set PWM servo angle
  Data:       servo_id (1-4), angle (0-180)
  Checksum:   (sum of all bytes + COMPLEMENT) & 0xFF, where COMPLEMENT = 1

  Bidirectional ESC mapping on servo PWM:
    angle  0  = full speed direction A
    angle 90  = neutral / stop
    angle 180 = full speed direction B
"""

import serial
import time as _time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# Rosmaster protocol constants
_HEAD = 0xFF
_DEVICE_ID = 0xFC
_COMPLEMENT = 257 - _DEVICE_ID  # 1
_FUNC_PWM_SERVO = 0x03
_ESC_NEUTRAL = 90  # angle that means stop for bidirectional ESCs


class YahboomDriverNode(Node):

    def __init__(self) -> None:
        super().__init__('yahboom_driver')

        # Serial port parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Robot geometry
        self.declare_parameter('wheel_base', 0.3)  # metres between left/right thrusters

        # Speed limits
        self.declare_parameter('max_speed', 1.0)  # m/s corresponding to ESC full deflection

        # Safety: auto-stop if no cmd_vel received within this many seconds
        self.declare_parameter('cmd_timeout', 0.5)

        # Update rate for sending serial commands
        self.declare_parameter('update_rate_hz', 20.0)

        # Dry-run mode: log commands without opening serial port
        self.declare_parameter('dry_run', False)

        # ESC arming duration — neutral sent for this many seconds on startup
        self.declare_parameter('esc_arm_duration', 3.0)

        self._serial = None
        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._armed = False

        dry_run = bool(self.get_parameter('dry_run').value)
        if not dry_run:
            self._open_serial()
            self._arm_escs()
        else:
            self._armed = True
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

    def _send_servo(self, servo_id: int, angle: int) -> None:
        """Send a single PWM servo command using the Rosmaster protocol."""
        angle = max(0, min(180, angle))
        cmd = [_HEAD, _DEVICE_ID, 0x00, _FUNC_PWM_SERVO, int(servo_id), int(angle)]
        cmd[2] = len(cmd) - 1
        checksum = sum(cmd, _COMPLEMENT) & 0xFF
        cmd.append(checksum)
        if self._serial and self._serial.is_open:
            self._serial.write(bytearray(cmd))

    def _arm_escs(self) -> None:
        """Send neutral (angle 90) for esc_arm_duration seconds to arm the ESCs."""
        if not self._serial or not self._serial.is_open:
            self.get_logger().warn('Cannot arm ESCs: serial port not open')
            return
        duration = float(self.get_parameter('esc_arm_duration').value)
        self.get_logger().info(f'Arming ESCs — sending neutral for {duration}s...')
        end = _time.monotonic() + duration
        while _time.monotonic() < end:
            try:
                self._send_servo(1, _ESC_NEUTRAL)
                self._send_servo(2, _ESC_NEUTRAL)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed during arming: {e}')
                return
            _time.sleep(0.05)
        self._armed = True
        self.get_logger().info('ESCs armed and ready')

    def _cmd_callback(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _send_command(self) -> None:
        if not self._armed:
            return
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

        # Map velocities to servo angles:
        #   velocity  0      -> angle 90  (neutral)
        #   velocity +max    -> angle 180 (full forward)
        #   velocity -max    -> angle 0   (full reverse)
        max_speed = float(self.get_parameter('max_speed').value)
        left_ratio = max(-1.0, min(1.0, left_vel / max_speed))
        right_ratio = max(-1.0, min(1.0, right_vel / max_speed))

        left_angle = int(_ESC_NEUTRAL + left_ratio * _ESC_NEUTRAL)
        right_angle = int(_ESC_NEUTRAL + right_ratio * _ESC_NEUTRAL)

        dry_run = bool(self.get_parameter('dry_run').value)
        if dry_run:
            self.get_logger().info(
                f'[DRY] L_angle={left_angle}  R_angle={right_angle}'
            )
        else:
            try:
                self._send_servo(1, left_angle)
                self._send_servo(2, right_angle)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')

        # Publish ESC values for UI / debugging (signed ratio * 90)
        esc_msg = Float32MultiArray()
        esc_msg.data = [float(left_angle - _ESC_NEUTRAL),
                        float(right_angle - _ESC_NEUTRAL)]
        self.esc_pub.publish(esc_msg)

    def destroy_node(self) -> None:
        # Send stop (neutral) repeatedly before shutting down
        if self._serial and self._serial.is_open:
            try:
                for _ in range(20):
                    self._send_servo(1, _ESC_NEUTRAL)
                    self._send_servo(2, _ESC_NEUTRAL)
                    _time.sleep(0.05)
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
