"""
Hardware bringup launch file.

Launches the real hardware stack:
  - yahboom_driver: serial bridge to ESCs via Yahboom board
  - mock_state_estimator: still used until real IMU/encoders are integrated
  - All planning / costmap / mission nodes from the sim stack

Usage:
  ros2 launch sim_bringup hw_bringup.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    dry_run = LaunchConfiguration('dry_run')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for the Yahboom CH340 board',
    )
    declare_dry_run = DeclareLaunchArgument(
        'dry_run', default_value='false',
        description='If true, log serial commands without sending',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('sim_bringup'), 'rviz', 'pool_sim.rviz']),
    )

    nodes = [
        # --- Hardware driver ---
        Node(
            package='yahboom_driver',
            executable='yahboom_driver_node',
            name='yahboom_driver',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': 115200,
                'wheel_base': 0.3,
                'max_speed': 1.0,
                'esc_max_value': 1000,
                'cmd_timeout': 0.5,
                'update_rate_hz': 20.0,
                'dry_run': dry_run,
            }],
            output='screen',
        ),
        # --- State estimator (mock until real IMU is integrated) ---
        Node(
            package='mock_state_estimator',
            executable='mock_state_estimator_node',
            name='mock_state_estimator',
            parameters=[{'update_rate_hz': 30.0}],
        ),
        # --- Mapping & planning (same as sim) ---
        Node(
            package='polar_to_grid_mapper',
            executable='polar_to_grid_mapper_node',
            name='polar_to_grid_mapper',
            parameters=[{'resolution': 0.15, 'publish_rate_hz': 2.0}],
        ),
        Node(
            package='coverage_planner',
            executable='coverage_planner_node',
            name='coverage_planner',
        ),
        Node(
            package='collision_costmap',
            executable='collision_costmap_node',
            name='collision_costmap',
        ),
        Node(
            package='local_planner',
            executable='local_planner_node',
            name='local_planner',
        ),
        Node(
            package='mission_executor',
            executable='mission_executor_node',
            name='mission_executor',
        ),
    ]

    return LaunchDescription([
        declare_serial_port,
        declare_dry_run,
        declare_rviz_config,
        *nodes,
    ])
