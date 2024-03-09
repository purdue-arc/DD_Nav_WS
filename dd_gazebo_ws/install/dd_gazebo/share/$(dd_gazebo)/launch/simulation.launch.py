import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file_path = os.path.join(get_package_share_directory(dd_gazebo), 'worlds', 'my_world.world')

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=[
                '-s', 'libgazebo_ros-factory.so',
                world_file_path
                ]
            ),
        ])
