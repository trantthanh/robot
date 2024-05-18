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
    

    diffrobot_maze_solve = Node(
        package = 'diffrobot_maze_solve',
        name = 'algorithm_node',
        executable='algorithm_node.py',
        output = 'screen'
    )

    # Run the node
    return LaunchDescription([
        diffrobot_simulation,
        diffrobot_lidarapp,
        diffrobot_maze_solve
    ])