import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mcl_localization')
    sim_share = get_package_share_directory('hero_sim')
    params = os.path.join(share, 'config', 'navigation.yaml')
    behavior_tree = os.path.join(share, 'config', 'navigate.xml')
    map_path = os.path.join(share, 'maps', 'obr_world.yaml')
    navigation_nodes = ['controller_server', 'planner_server', 'collision_monitor', 'bt_navigator']
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
            launch_arguments={'rviz': 'false', 'x': '-0.65', 'y': '-0.325',
                              'yaw': '0.0', 'mapping_lidar': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'localization.launch.py')),
               launch_arguments={'map': map_path, 'ground_truth_alignment': 'true', 'rviz': 'true'}.items(),
        ),
        Node(package='mcl_localization', executable='victim_node',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params, {'use_sim_time': True}],
             remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params, {'use_sim_time': True}], output='screen'),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[params, {'use_sim_time': True}], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, {'use_sim_time': True, 'default_nav_to_pose_bt_xml': behavior_tree}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': navigation_nodes}],
             output='screen'),
    ])