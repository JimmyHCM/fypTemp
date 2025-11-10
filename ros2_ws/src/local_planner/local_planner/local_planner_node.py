import math
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32


class LocalPlannerNode(Node):
    """Follows the global path using a pure-pursuit like controller."""

    def __init__(self) -> None:
        super().__init__('local_planner')

        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('min_linear_speed', 0.1)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('path_tolerance', 0.3)
        self.declare_parameter('reverse_allowed', False)

        self.path: List[PoseStamped] = []
        self.pose: Odometry | None = None
        self.velocity_scale = 1.0

        self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/collision_velocity_scale', self.velocity_scale_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update_cmd)

    def path_callback(self, msg: Path) -> None:
        self.path = list(msg.poses)

    def odom_callback(self, msg: Odometry) -> None:
        self.pose = msg

    def velocity_scale_callback(self, msg: Float32) -> None:
        self.velocity_scale = max(0.0, min(1.0, msg.data))

    def update_cmd(self) -> None:
        if not self.path or self.pose is None:
            self._publish_stop()
            return

        position = self.pose.pose.pose.position
        yaw = self._yaw_from_quaternion(self.pose.pose.pose.orientation)

        lookahead = float(self.get_parameter('lookahead_distance').value)
        target = self._find_lookahead_point(position.x, position.y, lookahead)
        if target is None:
            self._publish_stop()
            return

        dx = target.pose.position.x - position.x
        dy = target.pose.position.y - position.y
        transformed_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        transformed_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        curvature = 0.0
        if abs(transformed_x) > 1e-3 or abs(transformed_y) > 1e-3:
            curvature = (2.0 * transformed_y) / (lookahead ** 2)

        linear_speed = float(self.get_parameter('max_linear_speed').value) * self.velocity_scale
        linear_speed = max(float(self.get_parameter('min_linear_speed').value), linear_speed)
        angular_speed = max(-float(self.get_parameter('max_angular_speed').value),
                            min(float(self.get_parameter('max_angular_speed').value), curvature * linear_speed))

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        if not bool(self.get_parameter('reverse_allowed').value) and transformed_x < -self.get_parameter('path_tolerance').value:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def _find_lookahead_point(self, x: float, y: float, lookahead: float) -> PoseStamped | None:
        sq_lookahead = lookahead ** 2
        for pose in self.path:
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            if dx * dx + dy * dy >= sq_lookahead:
                return pose
        return self.path[-1] if self.path else None

    def _yaw_from_quaternion(self, orientation) -> float:
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
