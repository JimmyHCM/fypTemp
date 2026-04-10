"""
Yahboom YB-ERF01-V3.0 ESC Driver Node
======================================
Subscribes to /cmd_vel (Twist) and sends serial commands to the Yahboom
Rosmaster board to control two bidirectional ESCs on PWM servo ports S1 & S2
in a differential-drive configuration.

Serial protocol (Rosmaster V3.3.x):
  TX (host → board): Header 0xFF 0xFC
    Length:      total frame length - 1 (stored at byte index 2)
    Function:   0x03 = set PWM servo angle
    Data:       servo_id (1-4), angle (0-180)
    Checksum:   (sum of all bytes + COMPLEMENT) & 0xFF, where COMPLEMENT = 1

  RX (board → host): Header 0xFF 0xFB  (spontaneous stream)
    Length:      byte index 2; total frame = length + 2
    Function:   0x0E = ICM20948 IMU (18 data bytes)
                0x0C = orientation / Euler (6 data bytes)
                0x0D = velocity / encoder (16 data bytes)
                0x0A = system status (7 data bytes)
    Data:       varies by function
    Checksum:   sum(bytes[2:-1]) & 0xFF

  Bidirectional ESC mapping on servo PWM:
    angle  0  = full speed direction A
    angle 90  = neutral / stop
    angle 180 = full speed direction B
"""

import serial
import struct
import threading
import time as _time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray

# Rosmaster protocol constants
_HEAD = 0xFF
_DEVICE_ID_TX = 0xFC      # host → board
_DEVICE_ID_RX = 0xFB      # board → host
_COMPLEMENT = 257 - _DEVICE_ID_TX  # 1
_FUNC_PWM_SERVO = 0x03
_FUNC_IMU_DATA = 0x0E     # ICM20948 9-axis (18 data bytes)
_ESC_NEUTRAL = 90  # angle that means stop for bidirectional ESCs

# ICM20948 conversion factors
# Data arrives as 9x LE int16: [gx, gy, gz, ax, ay, az, mx, my, mz]
# Gyro:  values likely in centidegrees/s (stationary reads ~0.5 raw)
# Accel: firmware-processed, empirical ~9900 LSB/g
# Mag:   firmware-processed, empirical scale TBD
_GYRO_SCALE_CDPS = 100.0        # centidegrees/s → °/s
_ACCEL_SCALE_LSB_G = 9900.0     # LSB/g (empirical, measured flat)
_MAG_SCALE_UT = 0.15            # µT/LSB  (AK09916 nominal — may need tuning)
_GRAVITY = 9.80665              # m/s²
_DEG_TO_RAD = 3.14159265358979 / 180.0


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

        # IMU parameters
        self.declare_parameter('imu_enabled', True)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('imu_func_code', 0x0E)
        self.declare_parameter('accel_scale', 9900.0)
        self.declare_parameter('gyro_scale', 100.0)
        self.declare_parameter('dump_serial_frames', False)

        self._imu_enabled = bool(self.get_parameter('imu_enabled').value)
        self._imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self._imu_func = int(self.get_parameter('imu_func_code').value)
        self._accel_scale = float(self.get_parameter('accel_scale').value)
        self._gyro_scale = float(self.get_parameter('gyro_scale').value)
        self._dump_frames = bool(self.get_parameter('dump_serial_frames').value)

        self._serial = None
        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._armed = False
        self._reader_running = False

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

        # IMU publishers
        if self._imu_enabled:
            self._imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
            self._mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        else:
            self._imu_pub = None
            self._mag_pub = None

        # Start background serial reader for incoming data (IMU, etc.)
        if self._serial and self._serial.is_open:
            self._reader_running = True
            self._reader_thread = threading.Thread(
                target=self._serial_reader, daemon=True
            )
            self._reader_thread.start()

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
        cmd = [_HEAD, _DEVICE_ID_TX, 0x00, _FUNC_PWM_SERVO, int(servo_id), int(angle)]
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

    # ------------------------------------------------------------------
    # Serial reader — background thread parsing incoming Rosmaster frames
    # ------------------------------------------------------------------

    def _serial_reader(self) -> None:
        """Background thread: read and parse incoming 0xFF 0xFB frames."""
        buf = bytearray()
        while self._reader_running:
            try:
                if self._serial and self._serial.is_open and self._serial.in_waiting:
                    buf.extend(self._serial.read(self._serial.in_waiting))
                else:
                    _time.sleep(0.002)
                    continue
            except (serial.SerialException, OSError):
                _time.sleep(0.1)
                continue

            # Scan for complete RX frames (header 0xFF 0xFB)
            while len(buf) >= 5:
                # Find 0xFF header byte
                try:
                    idx = buf.index(_HEAD)
                except ValueError:
                    buf.clear()
                    break
                if idx > 0:
                    buf = buf[idx:]

                if len(buf) < 3:
                    break
                if buf[1] != _DEVICE_ID_RX:
                    buf = buf[1:]
                    continue

                frame_len_field = buf[2]
                total = frame_len_field + 2  # header(1) + device(1) already outside len
                if total < 5 or total > 64:
                    buf = buf[1:]
                    continue
                if len(buf) < total:
                    break  # incomplete, wait for more data

                frame = bytes(buf[:total])
                buf = buf[total:]

                # Checksum: sum of bytes[2:-1] & 0xFF
                expected = sum(frame[2:-1]) & 0xFF
                if frame[-1] != expected:
                    continue

                func = frame[3]
                data = frame[4:-1]
                self._handle_rx_frame(func, data)

    def _handle_rx_frame(self, func: int, data: bytes) -> None:
        """Dispatch a validated incoming frame by function code."""
        if self._dump_frames:
            self.get_logger().info(
                f'RX func=0x{func:02X} len={len(data)} data={data.hex()}'
            )

        if func == self._imu_func and self._imu_enabled:
            self._publish_imu(data)

    def _publish_imu(self, data: bytes) -> None:
        """Parse ICM20948 data from 0x0E frame and publish.

        Data layout (18 bytes, 9x LE int16):
          [0-5]   gyro:   gx, gy, gz  (centidegrees/s)
          [6-11]  accel:  ax, ay, az  (raw LSB, ~8192 LSB/g)
          [12-17] mag:    mx, my, mz  (raw AK09916, 0.15 µT/LSB)
        """
        now = self.get_clock().now().to_msg()

        if len(data) >= 12:
            gx_r, gy_r, gz_r, ax_r, ay_r, az_r = struct.unpack('<hhhhhh', data[:12])

            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = self._imu_frame_id
            msg.orientation_covariance[0] = -1.0  # orientation not estimated

            msg.angular_velocity.x = (gx_r / self._gyro_scale) * _DEG_TO_RAD
            msg.angular_velocity.y = (gy_r / self._gyro_scale) * _DEG_TO_RAD
            msg.angular_velocity.z = (gz_r / self._gyro_scale) * _DEG_TO_RAD

            msg.linear_acceleration.x = (ax_r / self._accel_scale) * _GRAVITY
            msg.linear_acceleration.y = (ay_r / self._accel_scale) * _GRAVITY
            msg.linear_acceleration.z = (az_r / self._accel_scale) * _GRAVITY

            self._imu_pub.publish(msg)

        # Magnetometer (bytes 12-17)
        if len(data) >= 18 and self._mag_pub:
            mx_r, my_r, mz_r = struct.unpack('<hhh', data[12:18])

            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = self._imu_frame_id
            mag.magnetic_field.x = mx_r * _MAG_SCALE_UT * 1e-6  # µT → T
            mag.magnetic_field.y = my_r * _MAG_SCALE_UT * 1e-6
            mag.magnetic_field.z = mz_r * _MAG_SCALE_UT * 1e-6

            self._mag_pub.publish(mag)

    # ------------------------------------------------------------------
    # cmd_vel handling
    # ------------------------------------------------------------------

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
        self._reader_running = False
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
