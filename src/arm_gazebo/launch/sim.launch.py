"""Gazebo + ros2_control + RViz2.  One command on /arm_controller/joint_trajectory
moves the arm in gz sim and (via /joint_states + TF) in RViz2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('arm_description')
    gz_pkg = get_package_share_directory('arm_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc, 'urdf', 'arm.urdf.xacro')
    controllers = os.path.join(desc, 'config', 'controllers.yaml')
    world = os.path.join(gz_pkg, 'worlds', 'arm_world.sdf')
    rviz_cfg = os.path.join(desc, 'rviz', 'arm.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=true controllers_file:=', controllers]),
        value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'robotic_arm',
                            '-world', 'arm_world'],
                 output='screen')

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                 output='screen')

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'])
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arm_controller', '--controller-manager-timeout', '60'])

    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        gz, rsp, clock, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        rviz,
    ])
