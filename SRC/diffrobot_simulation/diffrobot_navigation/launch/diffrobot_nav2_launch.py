import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file)
    )

    diffrobot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('diffrobot_simulation'), 'launch'), '/diffrobot_gazebo_launch.py']),
        )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
          )]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    

    diffrobot_lidarapp = Node(
        package='diffrobot_lidarapp',
        executable='scan_processor',
        parameters=[{'scan_raw_topic_name': '/scan_sim'}]
    )   
    # Configure the node
    diffrobot_navigation = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix = 'xterm -e',
        name='diffrobot_teleop'
    )

    # Run the node
    return LaunchDescription([
        diffrobot_simulation,
        diffrobot_navigation,
        diffrobot_lidarapp,
        rviz_launch,
        nav2_launch,
        slam_toolbox_node
    ])