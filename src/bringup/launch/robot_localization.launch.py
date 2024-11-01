import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    package_name = 'bringup'
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    robot_localization_file_path = os.path.join(pkg_share, 'param/ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='false', 
        description='Flag to disable use_sim_time'
    )

    imu_node = Node(
        package='robot_node1',
        executable='imu_node',
        name='mpu6500_node'

    )

    # Start robot localization using an Extended Kalman filter
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            robot_localization_file_path, 
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        imu_node,
        use_sim_time_arg,
        robot_localization_node
    ])