#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    LDS_LAUNCH_FILE = '/view_sllidar_a1_launch.py'

    usb_port_Arduino = LaunchConfiguration('usb_port_Arduino', default='/dev/ttyACM0')
    usb_port_Laptop = LaunchConfiguration('usb_port_Laptop', default='/dev/ttyUSB0')

    sllidar_pkg_dir = LaunchConfiguration(
        'sllidar_pkg_dir',
        default=os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port_Arduino',
            default_value=usb_port_Arduino,
            description='Connected USB port with Arduino'),

        DeclareLaunchArgument(
            'usb_port_Laptop',
            default_value=usb_port_Laptop,
            description='Connected USB port with Laptop'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robot_localization.launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sllidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={
                'port': usb_port_Laptop, 
                'frame_id': 'lidar_link',
                'scan_frequency': '5'}.items(),
        ),

    ])