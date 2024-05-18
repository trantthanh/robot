import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'diffrobot_description'
    file_subpath = 'urdf/diffrobot.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    diffrobot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': diffrobot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': diffrobot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    stat_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
    )

    diffrobot_lidarapp = Node(
        package='diffrobot_lidarapp',
        executable='scan_processor',
        parameters=[{'scan_raw_topic_name': '/scan'}]
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
        robot_state_publisher,
        joint_state_publisher,
        stat_tf,
        diffrobot_lidarapp,
        diffrobot_teleop
    ])