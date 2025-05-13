#!/usr/bin/env python3
#
#  … (üstteki telif ve Apache-2.0 satırları değişmedi) …
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = "burger"
    model_folder = f"turtlebot3_{TURTLEBOT3_MODEL}"

    # --- 1. LAUNCH ARGÜMANLARI -------------------------------------------------
    namespace      = LaunchConfiguration('namespace', default='')   # <-- yeni
    x_pose         = LaunchConfiguration('x_pose',  default='0.0')
    y_pose         = LaunchConfiguration('y_pose',  default='0.0')

    declare_ns_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='İsim alanı (robot1, robot2 …)')

    declare_x_cmd  = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Başlangıç X')

    declare_y_cmd  = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Başlangıç Y')

    # --- 2. ROBOT MODELİ --------------------------------------------------------
    sdf_path = os.path.join(
        get_package_share_directory('multiple_turtlebot3_sim'),
        'models', model_folder, 'model.sdf')

    spawn_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,                      # <-- yeni
        arguments=['-name', TURTLEBOT3_MODEL,
                   '-file', sdf_path,
                   '-x', x_pose, '-y', y_pose, '-z', '0.01'],
        output='screen')

    # --- 3. ROS <-> GZ KÖPRÜSÜ --------------------------------------------------
    bridge_yaml = os.path.join(
        get_package_share_directory('multiple_turtlebot3_sim'),
        'params', f'{model_folder}_bridge.yaml')   # dosyayı güncellediniz ✔

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,                      # <-- yeni
        parameters=[{'config_file': bridge_yaml}],
        output='screen')

    #  (burger modelinde kamera yok; varsa image_bridge ekleyebilirsiniz.)
    # ---------------------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(declare_ns_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)

    ld.add_action(spawn_cmd)
    ld.add_action(bridge_cmd)

    return ld
