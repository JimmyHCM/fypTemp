import math
from typing import List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import QoSPresetProfiles


class CollisionCostmapNode(Node):
    """Creates a forward-looking costmap and velocity scaling factor from sonar scans."""

    def __init__(self) -> None:
        super().__init__('collision_costmap')

        self.declare_parameter('sector_angle_deg', 120.0)
        self.declare_parameter('stop_distance', 0.4)
        self.declare_parameter('slow_distance', 1.2)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('grid_width', 40)
        self.declare_parameter('grid_height', 20)

        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)
        self.scale_pub = self.create_publisher(Float32, '/collision_velocity_scale', 10)
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(LaserScan, '/sonar/polar_scan', self.scan_callback, sensor_qos)

        self.get_logger().info('Collision costmap node online.')

    def scan_callback(self, msg: LaserScan) -> None:
        resolution = float(self.get_parameter('grid_resolution').value)
        width = int(self.get_parameter('grid_width').value)
        height = int(self.get_parameter('grid_height').value)
        sector_angle = math.radians(float(self.get_parameter('sector_angle_deg').value))
        half_sector = sector_angle / 2.0

        grid: List[int] = [-1 for _ in range(width * height)]

        min_distance = msg.range_max
        for angle_index, distance in enumerate(msg.ranges):
            if math.isnan(distance) or distance <= 0.0:
                continue
            angle = msg.angle_min + angle_index * msg.angle_increment
            if abs(angle) > half_sector:
                continue
            px = distance * math.cos(angle)
            py = distance * math.sin(angle)
            min_distance = min(min_distance, distance)
            gx = int(px / resolution)
            gy = int(py / resolution + height / 2)
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy * width + gx] = 100

        costmap = OccupancyGrid()
        costmap.header.frame_id = 'base_link'
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.info.resolution = resolution
        costmap.info.width = width
        costmap.info.height = height
        costmap.info.origin.position.x = 0.0
        costmap.info.origin.position.y = -0.5 * height * resolution
        costmap.info.origin.orientation.w = 1.0
        costmap.data = grid
        self.costmap_pub.publish(costmap)

        stop_distance = float(self.get_parameter('stop_distance').value)
        slow_distance = float(self.get_parameter('slow_distance').value)
        velocity_scale = 0.0
        if min_distance > slow_distance:
            velocity_scale = 1.0
        elif min_distance > stop_distance:
            velocity_scale = (min_distance - stop_distance) / (slow_distance - stop_distance)
        self.scale_pub.publish(Float32(data=float(max(0.0, min(1.0, velocity_scale)))))


def main() -> None:
    rclpy.init()
    node = CollisionCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
