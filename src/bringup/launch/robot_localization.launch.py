import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define path to custom ekf.yaml in bringup package
    declare_param_file = DeclareLaunchArgument(
        'param',
        default_value=os.path.join(
            get_package_share_directory('bringup'), 'param', 'ekf.yaml'
        ),
        description='Path to the YAML configuration file for the EKF'
    )

    # Load the custom parameter file path
    param_file = LaunchConfiguration('param')

    # Start IMU node (assuming a package and executable for imu_node)
    imu_node = Node(
        package='robot_node1',
        executable='imu_node',
        name='mpu6500_node'
    )

    # Include ekf.launch.py from robot_localization and pass param_file
    robot_localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_localization'),
                'launch',
                'ekf.launch.py'
            ])
        ]),
        launch_arguments={'param': param_file}.items()
    )

    return LaunchDescription([
        declare_param_file,
        imu_node,
        robot_localization_node
    ])
