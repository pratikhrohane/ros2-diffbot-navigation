import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('diff_robot_urdf'),
        'world',
        'home.world'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])
