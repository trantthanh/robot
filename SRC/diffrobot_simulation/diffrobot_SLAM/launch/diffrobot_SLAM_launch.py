import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node


def generate_launch_description():
    declare_map_save_path = DeclareLaunchArgument(
        name='map_save_path',
        default_value=os.path.join(get_package_share_directory('diffrobot_SLAM'), 'maps'),
        description='Path to save the generated map'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': 'true'}],
        arguments=['--ros-args', '--remap', '/map:=/map', '--remap', '/map_metadata:=/map_metadata', '__params:=/param.yaml'],
        remappings=[('/map', 'map'), ('/map_metadata', 'map_metadata')]
    )

    map_record_bag = Node(
        package='rosbag2_cli',
        executable='record',
        name='map_record_bag',
        namespace='rosbag2',
        output='screen',
        parameters=[{'use_sim_time': 'true'}],
        arguments=['-o', os.path.join(get_package_share_directory('my_slam_package'), 'maps'), '/map']
    )

    return LaunchDescription([
        declare_map_save_path,
        slam_toolbox_node,
        map_saver_node,
        map_record_bag
    ])

   