from enum import Enum, auto
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MissionState(Enum):
    IDLE = auto()
    PRESET = auto()
    AUTO_GEN = auto()
    MANUAL = auto()
    RETURN = auto()


class MissionExecutorNode(Node):
    """Simple mission state machine exposing services for mode transitions."""

    def __init__(self) -> None:
        super().__init__('mission_executor')

        self.state = MissionState.IDLE
        self.preset_path: Path | None = None
        self.latest_auto_path: Path | None = None
        self.manual_trace: List[PoseStamped] = []
        self.current_pose: Odometry | None = None

        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.manual_trace_pub = self.create_publisher(Path, '/manual_trace', 10)

        self.create_service(Trigger, '/mission/start_preset', self.handle_start_preset)
        self.create_service(Trigger, '/mission/gen_path', self.handle_gen_path)
        self.create_service(Trigger, '/mission/manual_on', self.handle_manual_on)
        self.create_service(Trigger, '/mission/manual_off', self.handle_manual_off)
        self.create_service(Trigger, '/mission/end_and_return', self.handle_end_and_return)
        self.create_service(Trigger, '/mission/save_manual_path', self.handle_save_manual_path)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(Path, '/coverage_plan', self.coverage_path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

        self.timer = self.create_timer(0.5, self.publish_state)

    def odom_callback(self, msg: Odometry) -> None:
        self.current_pose = msg
        if self.state == MissionState.MANUAL:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.manual_trace.append(pose)
            if len(self.manual_trace) > 2000:
                self.manual_trace = self.manual_trace[-2000:]

    def coverage_path_callback(self, msg: Path) -> None:
        self.latest_auto_path = msg
        if self.state == MissionState.AUTO_GEN:
            self.path_pub.publish(msg)

    def publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state.name))
        if self.manual_trace:
            path = Path()
            path.header.frame_id = self.manual_trace[0].header.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = list(self.manual_trace)
            self.manual_trace_pub.publish(path)

    def handle_start_preset(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.preset_path is None:
            response.success = False
            response.message = 'No preset path stored yet.'
            return response
        self.state = MissionState.PRESET
        self.path_pub.publish(self.preset_path)
        response.success = True
        response.message = 'Preset mission started.'
        return response

    def handle_gen_path(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.latest_auto_path is None:
            response.success = False
            response.message = 'No coverage path available.'
            return response
        self.state = MissionState.AUTO_GEN
        self.path_pub.publish(self.latest_auto_path)
        response.success = True
        response.message = 'Auto-gen coverage mission running.'
        return response

    def handle_manual_on(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.state = MissionState.MANUAL
        self.manual_trace.clear()
        self.path_pub.publish(Path())
        response.success = True
        response.message = 'Manual mode engaged; teleop takes control.'
        return response

    def handle_manual_off(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.state = MissionState.IDLE
        response.success = True
        response.message = 'Manual mode released; robot idle.'
        return response

    def handle_end_and_return(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.current_pose is None:
            response.success = False
            response.message = 'No odometry available to compute return path.'
            return response
        self.state = MissionState.RETURN
        path = Path()
        path.header.frame_id = self.current_pose.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        pose_now = PoseStamped()
        pose_now.header = path.header
        pose_now.pose = self.current_pose.pose.pose

        dock_pose = PoseStamped()
        dock_pose.header = path.header
        dock_pose.pose.position.x = 0.0
        dock_pose.pose.position.y = 0.0
        dock_pose.pose.orientation.w = 1.0

        path.poses = [pose_now, dock_pose]
        self.path_pub.publish(path)
        response.success = True
        response.message = 'Return-to-dock trajectory published.'
        return response

    def handle_save_manual_path(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if len(self.manual_trace) < 5:
            response.success = False
            response.message = 'Manual trace too short to save.'
            return response
        path = Path()
        path.header.frame_id = self.manual_trace[0].header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = list(self.manual_trace)
        self.preset_path = path
        response.success = True
        response.message = 'Manual path stored as preset.'
        return response


def main() -> None:
    rclpy.init()
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
