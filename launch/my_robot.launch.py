#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory('diff_robot_urdf'), 'launch'
    )

    # Delayed robot_state_publisher launch
    gz_robot_state_publisher = TimerAction(
        period=5.0,  # Delay by 5 seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
                )
            )
        ]
    )

    # Delayed spawn_entity launch
    gz_spawn_robot = TimerAction(
        period=5.0,  # Delay by 6 seconds 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'robot_spawn.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        gz_robot_state_publisher,
        gz_spawn_robot,
    ])
