import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('arm_control'), 'config', 'hardware_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(package='arm_control', executable='hardware_bridge', output='screen',
             parameters=[cfg, {'port': LaunchConfiguration('port'),
                               'dry_run': LaunchConfiguration('dry_run')}]),
    ])
