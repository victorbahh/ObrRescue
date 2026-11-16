import os
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    install_dir = get_package_prefix('mcl_localization')
    executable = os.path.join(install_dir, 'bin', 'mcl_node')

    return LaunchDescription([
        Node(
            package='mcl_localization',
            executable=executable,
            name='mcl_node',
            output='screen',
        ),
    ])
