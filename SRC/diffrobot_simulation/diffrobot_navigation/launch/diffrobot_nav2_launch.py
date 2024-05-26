import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define package share directories
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_diffrobot_simulation_dir = get_package_share_directory('diffrobot_simulation')
    pkg_diffrobot_navigation_dir = get_package_share_directory('diffrobot_navigation')

    # Paths to launch files and parameter files
    nav2_launch_file = os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
    diffrobot_gazebo_launch_file = os.path.join(pkg_diffrobot_simulation_dir, 'launch', 'diffrobot_gazebo_launch.py')
    nav2_params_file = os.path.join(pkg_diffrobot_navigation_dir, 'params', 'nav2_sim_params.yaml')

    # Verify paths
    if not os.path.isfile(nav2_launch_file):
        raise FileNotFoundError(f"Nav2 launch file not found: {nav2_launch_file}")
    if not os.path.isfile(diffrobot_gazebo_launch_file):
        raise FileNotFoundError(f"Gazebo launch file not found: {diffrobot_gazebo_launch_file}")
    if not os.path.isfile(nav2_params_file):
        raise FileNotFoundError(f"Nav2 params file not found: {nav2_params_file}")

    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=nav2_params_file,
        description='Full path to map yaml file to load'
    )

    # Include Gazebo launch file
    diffrobot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diffrobot_gazebo_launch_file),
        launch_arguments={'map': LaunchConfiguration('map')}.items()
    )

    # Include Nav2 bringup launch file with map argument
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={'map': LaunchConfiguration('map')}.items()
    )

    # RViz2 node
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d', os.path.join(
                pkg_nav2_dir,
                'rviz',
                'nav2_default_view.rviz'
            )
        ],
        output='screen'
    )

    # SLAM toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # Lidar app node
    diffrobot_lidarapp = Node(
        package='diffrobot_lidarapp',
        executable='scan_processor',
        name='scan_processor',
        parameters=[{'scan_raw_topic_name': '/scan_sim'}],
        output='screen'
    )

    # Teleop node
    diffrobot_navigation = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='diffrobot_teleop',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        map_arg,
        diffrobot_simulation,
        nav2_launch,
        rviz_launch,
        slam_toolbox_node,
        diffrobot_lidarapp,
        diffrobot_navigation
    ])
