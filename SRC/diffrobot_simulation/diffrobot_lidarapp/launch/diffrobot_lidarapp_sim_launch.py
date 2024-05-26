import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Define package share directories
    pkg_diffrobot_simulation_dir = get_package_share_directory('diffrobot_simulation')
    pkg_nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to RViz configuration file
    rviz_config_file = os.path.join(pkg_nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # Verify RViz configuration file path
    if not os.path.isfile(rviz_config_file):
        raise FileNotFoundError(f"RViz config file not found: {rviz_config_file}")

    # Include Gazebo launch file
    diffrobot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_diffrobot_simulation_dir, 'launch'), '/diffrobot_gazebo_launch.py'])
    )

    # RViz2 node
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Lidar app node
    diffrobot_lidarapp = Node(
        package='diffrobot_lidarapp',
        executable='scan_processor',
        parameters=[{'scan_raw_topic_name': '/scan_sim'}],
        output='screen'
    )

    # Teleop node
    diffrobot_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='diffrobot_teleop',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        diffrobot_simulation,
        rviz_launch,
        diffrobot_lidarapp,
        diffrobot_teleop
    ])
