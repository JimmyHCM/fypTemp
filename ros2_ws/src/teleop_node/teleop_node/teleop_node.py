import sys
import termios
import tty
from contextlib import contextmanager

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

KEY_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
}


@contextmanager
def raw_terminal(fd):
    settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


class TeleopNode(Node):
    """Very small keyboard teleop node for quick manual control."""

    def __init__(self) -> None:
        super().__init__('teleop_node')

        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.7)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.poll_keyboard)
        self.stop_sent = False

        self.get_logger().info('Keyboard teleop ready: w/s forward/back, a/d yaw, space stop, q quit.')

    def poll_keyboard(self) -> None:
        linear_speed = float(self.get_parameter('linear_speed').value)
        angular_speed = float(self.get_parameter('angular_speed').value)

        if not sys.stdin.isatty():
            return

        with raw_terminal(sys.stdin.fileno()):
            cmd = self._read_key()
        if cmd is None:
            return

        twist = Twist()
        if cmd == ' ':
            self.cmd_pub.publish(twist)
            self.stop_sent = True
            return
        if cmd in ('q', '\x03'):
            rclpy.shutdown()
            return
        if cmd in KEY_BINDINGS:
            lin_scale, ang_scale = KEY_BINDINGS[cmd]
            twist.linear.x = lin_scale * linear_speed
            twist.angular.z = ang_scale * angular_speed
            self.cmd_pub.publish(twist)
            self.stop_sent = False
        elif not self.stop_sent:
            self.cmd_pub.publish(twist)
            self.stop_sent = True

    def _read_key(self) -> str | None:
        import select

        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None


def main() -> None:
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
