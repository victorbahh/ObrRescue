"""HeRo in a Gazebo Harmonic world.

ros2 launch hero_sim sim.launch.py [world:=obr_rescue|hero_arena|/path/to.sdf]
    [gui:=true] [rviz:=true] [x:=0.0] [y:=0.0] [yaw:=0.0] [mapping_lidar:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_world(share, value):
    """A world name from hero_sim/worlds (without .sdf), or a path to any .sdf file."""
    if value.endswith('.sdf'):
        path = os.path.abspath(os.path.expanduser(value))
    else:
        path = os.path.join(share, 'worlds', value + '.sdf')
    if not os.path.isfile(path):
        available = sorted(f[:-4] for f in os.listdir(os.path.join(share, 'worlds')) if f.endswith('.sdf'))
        raise RuntimeError(f"World '{value}' not found ({path}). Available: {', '.join(available)}")
    return path


def launch_gazebo(context, share, gui_config):
    world = resolve_world(share, LaunchConfiguration('world').perform(context))
    server_only = LaunchConfiguration('gui').perform(context).lower() != 'true'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ('-s ' if server_only else '') + f'-r --gui-config {gui_config} {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )]


def generate_launch_description():
    share = get_package_share_directory('hero_sim')
    gui_config = os.path.join(get_package_share_directory('hero_jazzy'), 'config', 'gui.config')
    xacro_file = os.path.join(share, 'urdf', 'hero_sim.urdf.xacro')
    bridge_config = os.path.join(share, 'config', 'bridge.yaml')
    rviz_config = os.path.join(share, 'rviz', 'hero.rviz')

    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' mapping_lidar:=', LaunchConfiguration('mapping_lidar')]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='obr_rescue',
                              description='World name in hero_sim/worlds (obr_rescue, hero_arena) or a .sdf path'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        DeclareLaunchArgument('mapping_lidar', default_value='false',
                      description='Use a simulated 360-degree lidar for mapping'),

        OpaqueFunction(function=launch_gazebo, args=[share, gui_config]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'hero', '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                '-z', '0.012', '-Y', LaunchConfiguration('yaw'),
            ],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
