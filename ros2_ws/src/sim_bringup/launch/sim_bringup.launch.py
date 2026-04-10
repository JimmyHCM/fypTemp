from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scenario = LaunchConfiguration('scenario')
    teleop = LaunchConfiguration('teleop')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_scenario = DeclareLaunchArgument('scenario', default_value='rect_pool')
    declare_teleop = DeclareLaunchArgument('teleop', default_value='true')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('sim_bringup'), 'rviz', 'pool_sim.rviz']),
    )

    nodes = [
        Node(
            package='odom_integrator',
            executable='odom_integrator_node',
            name='odom_integrator',
            parameters=[{'update_rate_hz': 30.0}],
        ),
        Node(
            package='sim_scan_publisher',
            executable='sim_scan_publisher_node',
            name='sim_scan_publisher',
            parameters=[{'scenario': scenario}],
        ),
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
        Node(
            package='teleop_node',
            executable='teleop_node',
            name='teleop_node',
            emulate_tty=True,
            condition=IfCondition(teleop),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ]

    return LaunchDescription([declare_scenario, declare_teleop, declare_rviz_config, *nodes])
