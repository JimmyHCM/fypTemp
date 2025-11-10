import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class CoveragePlannerNode(Node):
    """Generates a boustrophedon coverage path across the free portion of the map."""

    def __init__(self) -> None:
        super().__init__('coverage_planner')

        self.declare_parameter('lane_spacing', 0.75)
        self.declare_parameter('overlap', 0.15)
        self.declare_parameter('free_threshold', 30)
        self.declare_parameter('min_area_cells', 50)

        self.map: OccupancyGrid | None = None

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_pub = self.create_publisher(Path, '/coverage_plan', 10)

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map = msg
        path = self._plan_path(msg)
        if path is None:
            return
        self.path_pub.publish(path)

    def _plan_path(self, occupancy: OccupancyGrid) -> Path | None:
        width = occupancy.info.width
        height = occupancy.info.height
        data = occupancy.data

        free_indices = [i for i, value in enumerate(data) if value >= 0 and value <= int(self.get_parameter('free_threshold').value)]
        if len(free_indices) < int(self.get_parameter('min_area_cells').value):
            self.get_logger().warn_once('Not enough free cells for coverage planning yet.')
            return None

        xs, ys = zip(*[self._index_to_xy(index, width) for index in free_indices])
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        resolution = occupancy.info.resolution
        origin_x = occupancy.info.origin.position.x
        origin_y = occupancy.info.origin.position.y

        lane_spacing = max(resolution, float(self.get_parameter('lane_spacing').value) - float(self.get_parameter('overlap').value))
        lane_spacing_cells = max(1, int(lane_spacing / resolution))

        poses: List[PoseStamped] = []
        direction = 1

        for row in range(min_y, max_y + 1, lane_spacing_cells):
            y_world = origin_y + (row + 0.5) * resolution
            start_x = origin_x + (min_x + 0.5) * resolution
            end_x = origin_x + (max_x + 0.5) * resolution
            if direction < 0:
                start_x, end_x = end_x, start_x
            poses.append(self._make_pose(start_x, y_world, direction))
            poses.append(self._make_pose(end_x, y_world, direction))
            direction *= -1

        if not poses:
            return None

        path = Path()
        path.header.frame_id = occupancy.header.frame_id or 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = poses
        return path

    def _make_pose(self, x: float, y: float, direction: int) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw = 0.0 if direction >= 0 else math.pi
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _index_to_xy(self, index: int, width: int) -> Tuple[int, int]:
        y = index // width
        x = index % width
        return x, y


def main() -> None:
    rclpy.init()
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
