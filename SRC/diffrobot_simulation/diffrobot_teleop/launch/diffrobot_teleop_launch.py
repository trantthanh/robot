import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node


def generate_launch_description():
    
    # Configure the node
    diffrobot_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix = 'xterm -e',
        name='diffrobot_teleop'
    )

    # Run the node
    return LaunchDescription([
        diffrobot_teleop
    ])