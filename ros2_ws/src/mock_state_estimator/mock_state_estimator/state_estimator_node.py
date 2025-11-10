import math
import random
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_drift: float = 0.0
    angular_drift: float = 0.0


class MockStateEstimatorNode(Node):
    """Integrates commanded velocity into an odometry stream with configurable noise."""

    def __init__(self) -> None:
        super().__init__('mock_state_estimator')

        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('linear_velocity_noise_std', 0.02)
        self.declare_parameter('angular_velocity_noise_std', 0.01)
        self.declare_parameter('linear_drift_std', 0.0005)
        self.declare_parameter('angular_drift_std', 0.0002)
        self.declare_parameter('enable_drift', True)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.state = RobotState()
        self.last_cmd = Twist()
        self.last_time = self.get_clock().now()

        update_period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.timer = self.create_timer(update_period, self.update_state)

        self.get_logger().info('Mock state estimator running (update_rate=%.1f Hz)' % (1.0 / update_period))

    def cmd_callback(self, msg: Twist) -> None:
        self.last_cmd = msg

    def update_state(self) -> None:
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = current_time

        lin_vel = self.last_cmd.linear.x + random.gauss(0.0, float(self.get_parameter('linear_velocity_noise_std').value))
        ang_vel = self.last_cmd.angular.z + random.gauss(0.0, float(self.get_parameter('angular_velocity_noise_std').value))

        self.state.linear_drift += random.gauss(0.0, float(self.get_parameter('linear_drift_std').value))
        self.state.angular_drift += random.gauss(0.0, float(self.get_parameter('angular_drift_std').value))

        enable_drift = bool(self.get_parameter('enable_drift').value)
        drift_linear = self.state.linear_drift if enable_drift else 0.0
        drift_angular = self.state.angular_drift if enable_drift else 0.0

        self.state.yaw += (ang_vel + drift_angular) * dt
        self.state.x += (lin_vel + drift_linear) * math.cos(self.state.yaw) * dt
        self.state.y += (lin_vel + drift_linear) * math.sin(self.state.yaw) * dt

        quat = self._yaw_to_quaternion(self.state.yaw)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.state.x
        odom.pose.pose.position.y = self.state.y
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = lin_vel
        odom.twist.twist.angular.z = ang_vel

        pose = PoseWithCovarianceStamped()
        pose.header = odom.header
        pose.pose = odom.pose

        self.odom_pub.publish(odom)
        self.pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self.state.x
        transform.transform.translation.y = self.state.y
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down mock state estimator.')
        super().destroy_node()

    def _yaw_to_quaternion(self, yaw: float) -> tuple[float, float, float, float]:
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))


def main() -> None:
    rclpy.init()
    node = MockStateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
