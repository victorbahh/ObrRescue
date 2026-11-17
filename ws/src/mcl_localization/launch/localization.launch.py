import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('mcl_localization'),
                               'rviz', 'localization.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the saved map YAML'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ground_truth_alignment', default_value='false'),
        DeclareLaunchArgument('world_x', default_value='0.0'),
        DeclareLaunchArgument('world_y', default_value='0.0'),
        DeclareLaunchArgument('world_yaw', default_value='0.0'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'use_sim_time': True,
                         'yaml_filename': ParameterValue(LaunchConfiguration('map'), value_type=str)}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}],
            output='screen',
        ),
        Node(package='mcl_localization', executable='mcl_node', name='mcl_node',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', LaunchConfiguration('world_x'), '--y', LaunchConfiguration('world_y'),
                       '--z', '0', '--yaw', LaunchConfiguration('world_yaw'), '--pitch', '0',
                       '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'ground_truth_world'],
            condition=IfCondition(LaunchConfiguration('ground_truth_alignment')),
            output='screen',
        ),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])