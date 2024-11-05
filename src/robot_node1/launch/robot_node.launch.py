from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():

#     navigation_launch_file = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             get_package_share_directory('robot_localization'),
#             '/launch/ekf.launch.py'
#         ])
#     )


#     return LaunchDescription([
        
#         Node(
#             package='robot_localization',
#             executable='ekf_node',
#             name='ekf_filter_node',
#             output='screen',
#             parameters=['/path_to_your_config/ekf_params.yaml'],
#             remappings=[
#                 ('/odom', '/odom'),  # remap odom 话题
#                 ('/imu/data', '/imu/data')  # remap IMU 话题
#             ]
#         )
#     ])

