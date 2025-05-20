#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('diff_robot_urdf'),
        'urdf',
        'diff_bot.urdf'
    )

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.2')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position for the robot'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position for the robot'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_bot',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_x_position_cmd,
        declare_y_position_cmd,
        spawn_entity_cmd
    ])
