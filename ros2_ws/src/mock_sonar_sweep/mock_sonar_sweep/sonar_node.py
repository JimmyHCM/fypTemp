import math
import random
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


@dataclass
class Segment:
    start: Tuple[float, float]
    end: Tuple[float, float]


class MockSonarSweepNode(Node):
    """Publishes synthetic LaserScan messages emulating a scanning sonar."""

    def __init__(self) -> None:
        super().__init__('mock_sonar_sweep')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_pub = self.create_publisher(LaserScan, '/sonar/polar_scan', qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)

        self.declare_parameter('fov_deg', 150.0)
        self.declare_parameter('num_beams', 181)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('scenario', 'rect_pool')
        self.declare_parameter('noise_std', 0.05)
        self.declare_parameter('dropout_prob', 0.05)
        self.declare_parameter('spurious_prob', 0.02)
        self.declare_parameter('spurious_min_range', 0.3)
        self.declare_parameter('spurious_max_range', 1.5)
        self.declare_parameter('update_rate_hz', 5.0)

        self.pose: Pose | None = None
        self.pool_segments = self._build_environment(str(self.get_parameter('scenario').value))

        update_period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.timer = self.create_timer(update_period, self.publish_scan)

        self.get_logger().info('Mock sonar sweep node ready (scenario=%s)' % self.get_parameter('scenario').value)

    def odom_callback(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    def publish_scan(self) -> None:
        if not self.scan_pub.get_subscription_count():
            return

        pose = self.pose or Pose()
        beam_count = int(self.get_parameter('num_beams').value)
        max_range = float(self.get_parameter('max_range').value)
        fov_rad = math.radians(float(self.get_parameter('fov_deg').value))
        angle_min = -0.5 * fov_rad
        angle_max = 0.5 * fov_rad
        angle_increment = (angle_max - angle_min) / (beam_count - 1)

        ranges = []
        intensities = []

        yaw = self._yaw_from_pose(pose)
        position = (pose.position.x, pose.position.y)

        for i in range(beam_count):
            bearing = angle_min + i * angle_increment
            distance, intensity = self._ray_cast(position, yaw + bearing, max_range)

            noisy_distance = self._apply_noise(distance, max_range)
            if noisy_distance is None:
                ranges.append(float('nan'))
                intensities.append(0.0)
            else:
                ranges.append(noisy_distance)
                intensities.append(intensity)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.range_min = 0.1
        scan.range_max = max_range
        scan.ranges = ranges
        scan.intensities = intensities

        self.scan_pub.publish(scan)

    def _yaw_from_pose(self, pose: Pose) -> float:
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w or 1.0
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def _apply_noise(self, distance: float, max_range: float) -> float | None:
        if math.isinf(distance):
            distance = max_range
        if random.random() < float(self.get_parameter('dropout_prob').value):
            return None
        noisy = distance + random.gauss(0.0, float(self.get_parameter('noise_std').value))
        if random.random() < float(self.get_parameter('spurious_prob').value):
            noisy = random.uniform(
                float(self.get_parameter('spurious_min_range').value),
                float(self.get_parameter('spurious_max_range').value),
            )
        noisy = max(0.05, min(noisy, max_range))
        return noisy

    def _ray_cast(self, origin: Tuple[float, float], heading: float, max_range: float) -> Tuple[float, float]:
        dx = math.cos(heading)
        dy = math.sin(heading)
        min_distance = float('inf')
        intensity = 1.0

        for segment in self.pool_segments:
            distance = self._ray_to_segment(origin, (dx, dy), segment)
            if distance is not None and 0.0 < distance < min_distance:
                min_distance = distance
                intensity = max(0.2, 1.0 - distance / max_range)

        if min_distance == float('inf') or min_distance > max_range:
            return max_range, 0.1
        return min_distance, intensity

    def _ray_to_segment(
        self,
        origin: Tuple[float, float],
        direction: Tuple[float, float],
        segment: Segment,
    ) -> float | None:
        x1, y1 = origin
        dx, dy = direction
        x3, y3 = segment.start
        x4, y4 = segment.end

        denom = (dx * (y3 - y4) - dy * (x3 - x4))
        if abs(denom) < 1e-6:
            return None

        t = ((x3 - x1) * (y3 - y4) - (y3 - y1) * (x3 - x4)) / denom
        u = -((dx) * (y3 - y1) - (dy) * (x3 - x1)) / denom

        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def _build_environment(self, scenario: str) -> List[Segment]:
        if scenario == 'l_pool':
            return self._l_pool_segments()
        if scenario == 'island_pool':
            return self._island_pool_segments()
        return self._rect_pool_segments()

    def _rect_pool_segments(self) -> List[Segment]:
        width = 10.0
        height = 6.0
        return self._axis_aligned_rectangle_segments(width, height)

    def _l_pool_segments(self) -> List[Segment]:
        outer = [
            Segment((-5.0, -3.0), (5.0, -3.0)),
            Segment((5.0, -3.0), (5.0, 3.0)),
            Segment((5.0, 3.0), (0.0, 3.0)),
            Segment((0.0, 3.0), (0.0, 1.0)),
            Segment((0.0, 1.0), (-5.0, 1.0)),
            Segment((-5.0, 1.0), (-5.0, -3.0)),
        ]
        inner_steps = self._axis_aligned_rectangle_segments(1.5, 0.8, center=(2.0, 2.0))
        return outer + inner_steps

    def _island_pool_segments(self) -> List[Segment]:
        pool = self._axis_aligned_rectangle_segments(10.0, 6.0)
        island = self._axis_aligned_rectangle_segments(2.0, 1.2, center=(0.0, 0.5))
        steps = self._axis_aligned_rectangle_segments(3.0, 0.7, center=(-3.5, -2.0))
        return pool + island + steps

    def _axis_aligned_rectangle_segments(
        self,
        width: float,
        height: float,
        center: Tuple[float, float] = (0.0, 0.0),
    ) -> List[Segment]:
        cx, cy = center
        half_w = width / 2.0
        half_h = height / 2.0
        top_left = (cx - half_w, cy + half_h)
        top_right = (cx + half_w, cy + half_h)
        bottom_left = (cx - half_w, cy - half_h)
        bottom_right = (cx + half_w, cy - half_h)
        return [
            Segment(bottom_left, bottom_right),
            Segment(bottom_right, top_right),
            Segment(top_right, top_left),
            Segment(top_left, bottom_left),
        ]


def main() -> None:
    rclpy.init()
    node = MockSonarSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
