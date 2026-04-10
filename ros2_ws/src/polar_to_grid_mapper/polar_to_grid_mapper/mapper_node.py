import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSPresetProfiles


class PolarToGridMapperNode(Node):
    """Converts laser scans into an occupancy grid via basic ray casting."""

    def __init__(self) -> None:
        super().__init__('polar_to_grid_mapper')

        self.declare_parameter('resolution', 0.15)
        self.declare_parameter('width', 200)  # cells
        self.declare_parameter('height', 200)
        self.declare_parameter('hit_log_odds', 0.62)
        self.declare_parameter('miss_log_odds', -0.43)
        self.declare_parameter('min_log_odds', -2.0)
        self.declare_parameter('max_log_odds', 3.5)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('scan_topic', '/scan')

        self.pose: Pose | None = None
        self.map_frame = 'map'
        self.base_frame = 'base_link'

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.map_origin = (-self.width * self.resolution / 2.0, -self.height * self.resolution / 2.0, 0.0)
        self.log_odds = [0.0 for _ in range(self.width * self.height)]

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        scan_topic = str(self.get_parameter('scan_topic').value)

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

        publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(publish_period, self.publish_map)

        self.get_logger().info('Polar to grid mapper ready (resolution=%.2fm)' % self.resolution)

    def odom_callback(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan) -> None:
        pose = self.pose or Pose()
        yaw = self._yaw_from_pose(pose)
        robot_x, robot_y = pose.position.x, pose.position.y

        hit_log = float(self.get_parameter('hit_log_odds').value)
        miss_log = float(self.get_parameter('miss_log_odds').value)
        min_log = float(self.get_parameter('min_log_odds').value)
        max_log = float(self.get_parameter('max_log_odds').value)

        max_range = msg.range_max
        for idx, raw_range in enumerate(msg.ranges):
            if math.isnan(raw_range):
                continue
            r = min(raw_range, max_range)
            angle = yaw + msg.angle_min + idx * msg.angle_increment
            end_x = robot_x + r * math.cos(angle)
            end_y = robot_y + r * math.sin(angle)
            cells = self._bresenham(robot_x, robot_y, end_x, end_y)

            for free_index in cells[:-1]:
                self.log_odds[free_index] = max(min_log, self.log_odds[free_index] + miss_log)

            if r < max_range - 1e-3 and cells:
                hit_index = cells[-1]
                self.log_odds[hit_index] = min(max_log, self.log_odds[hit_index] + hit_log)

    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.map_origin[0]
        msg.info.origin.position.y = self.map_origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = [self._log_odds_to_probability(value) for value in self.log_odds]
        self.map_pub.publish(msg)

    def _log_odds_to_probability(self, value: float) -> int:
        probability = 1.0 - (1.0 / (1.0 + math.exp(value)))
        occ_value = int(probability * 100.0)
        occ_value = max(0, min(100, occ_value))
        if abs(value) < 1e-3:
            return -1
        return occ_value

    def _bresenham(self, start_x: float, start_y: float, end_x: float, end_y: float) -> List[int]:
        start = self._world_to_grid(start_x, start_y)
        end = self._world_to_grid(end_x, end_y)
        return self._bresenham_indices(start, end)

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.map_origin[0]) / self.resolution)
        gy = int((y - self.map_origin[1]) / self.resolution)
        return gx, gy

    def _bresenham_indices(self, start: Tuple[int, int], end: Tuple[int, int]) -> List[int]:
        x0, y0 = start
        x1, y1 = end
        points: List[int] = []

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            if 0 <= x0 < self.width and 0 <= y0 < self.height:
                points.append(y0 * self.width + x0)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

        return points

    def _yaw_from_pose(self, pose: Pose) -> float:
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w or 1.0
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main() -> None:
    rclpy.init()
    node = PolarToGridMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
