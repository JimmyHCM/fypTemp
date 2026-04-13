"""
Hardware bringup launch file.

Launches the real hardware stack:
  - yahboom_driver: serial bridge to ESCs via Yahboom board
  - sllidar_ros2: RPLIDAR C1 driver publishing /scan
  - yahboom_driver: also reads ICM20948 IMU data from the board (publishes /imu/data_raw, /imu/mag)
  - odom_integrator: still used until real encoders are integrated
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
    lidar_port = LaunchConfiguration('lidar_port')

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
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB1',
        description='Serial port for the RPLIDAR C1',
    )

    nodes = [
        # --- RPLIDAR C1 ---
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),
        # --- Static TF: base_link → laser ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser'],
        ),
        # --- SLAM Toolbox (online async) — publishes map→odom TF and /map ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                PathJoinSubstitution([FindPackageShare('sim_bringup'), 'config', 'slam_toolbox_online_async.yaml']),
            ],
            output='screen',
        ),
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
                'imu_enabled': True,
                'imu_frame_id': 'imu_link',
                'dump_serial_frames': False,
            }],
            output='screen',
        ),
        # --- Static TF: base_link → imu_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        # --- Odometry integrator (until real encoders are integrated) ---
        Node(
            package='odom_integrator',
            executable='odom_integrator_node',
            name='odom_integrator',
            parameters=[{'update_rate_hz': 30.0}],
        ),
        # --- Planning ---
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
        # --- RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ]

    return LaunchDescription([
        declare_serial_port,
        declare_dry_run,
        declare_rviz_config,
        declare_lidar_port,
        *nodes,
    ])
