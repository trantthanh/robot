import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node


def generate_launch_description():

    diffrobot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('diffrobot_simulation'), 'launch'), '/diffrobot_gazebo_launch.py']),
        )
    
    diffrobot_lidarapp = Node(
        package='diffrobot_lidarapp',
        executable='scan_processor',
        parameters=[{'scan_raw_topic_name': '/scan_sim'}]
    )
    
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
        diffrobot_simulation,
        diffrobot_lidarapp,
        diffrobot_teleop
    ])