import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command ,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = FindPackageShare(package = 'robot_description').find('robot_description')
    default_model_path = os.path.join(
        pkg_share, 
        'urdf/robot.urdf'
    )

    model = LaunchConfiguration('model')
    use_sim_time_arg = LaunchConfiguration('use_sim_time')

    model_launch_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        model_launch_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])