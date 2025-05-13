#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share      = get_package_share_directory('multiple_turtlebot3_sim')
    slam_pkg_share = get_package_share_directory('slam_toolbox')

    # Path to your params
    slam_params_file = os.path.join(pkg_share, 'params', 'slam_toolbox.yaml')

    # Use the lifecycle-aware sync launch
    sync_launch = os.path.join(
        slam_pkg_share, 'launch', 'online_sync_launch.py'
    )

    # allow toggling sim time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sync_launch),
        launch_arguments={
            'use_sim_time':     LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_file,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_launch,
    ])
