#!/usr/bin/env python3
"""
Manual Control Logger
=====================
Subscribes to /cmd_vel and /esc_values and writes CSV logs for tuning
the joystick-to-ESC pipeline.

Usage:
  source install/setup.bash
  python3 log_manual_control.py

Logs are written to  ros2_ws/log/manual_tuning/<timestamp>.csv
Press Ctrl+C to stop. A summary is printed on exit.
"""

import os
import csv
import signal
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String


class ManualControlLogger(Node):

    def __init__(self, log_path: Path) -> None:
        super().__init__('manual_control_logger')

        self.log_path = log_path
        self._csv_file = open(log_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            'timestamp',
            'elapsed_s',
            'mission_state',
            'cmd_linear_x',   # joystick forward/back  (m/s)
            'cmd_angular_z',  # joystick left/right    (rad/s)
            'esc_left',       # ESC offset for left motor  (-90..+90)
            'esc_right',      # ESC offset for right motor (-90..+90)
        ])

        # Latest values (merged on each callback)
        self._mission_state = '?'
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._esc_left = 0.0
        self._esc_right = 0.0
        self._row_count = 0
        self._start_time = self.get_clock().now()
        self._webapp_connected = False

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/esc_values', self._esc_values_cb, 10
        )
        self.create_subscription(String, '/mission_state', self._state_cb, 10)

        # Flush timer — write buffered data every second
        self.create_timer(1.0, self._flush)

        self.get_logger().info(f'Logging to {log_path}')
        self.get_logger().info('Waiting for /cmd_vel and /esc_values messages…')

    # ---- callbacks -------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist) -> None:
        if not self._webapp_connected:
            self._webapp_connected = True
            self.get_logger().info(
                'First /cmd_vel received — webapp appears connected. Logging started.'
            )
        self._linear_x = msg.linear.x
        self._angular_z = msg.angular.z
        self._write_row()

    def _esc_values_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self._esc_left = msg.data[0]
            self._esc_right = msg.data[1]
        self._write_row()

    def _state_cb(self, msg: String) -> None:
        self._mission_state = msg.data

    # ---- CSV helpers -----------------------------------------------------

    def _write_row(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds / 1e9
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self._writer.writerow([
            stamp,
            f'{elapsed:.3f}',
            self._mission_state,
            f'{self._linear_x:.4f}',
            f'{self._angular_z:.4f}',
            f'{self._esc_left:.1f}',
            f'{self._esc_right:.1f}',
        ])
        self._row_count += 1

    def _flush(self) -> None:
        self._csv_file.flush()

    # ---- shutdown --------------------------------------------------------

    def destroy_node(self) -> None:
        self._csv_file.close()
        self.get_logger().info(
            f'Log closed — {self._row_count} rows written to {self.log_path}'
        )
        super().destroy_node()


def main() -> None:
    rclpy.init()

    # Create log directory
    log_dir = Path(__file__).resolve().parent / 'log' / 'manual_tuning'
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_path = log_dir / f'{stamp}.csv'

    node = ManualControlLogger(log_path)

    # Graceful shutdown on Ctrl+C
    def _shutdown(sig, frame):
        node.get_logger().info('Shutting down…')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
