import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command

def generate_launch_description():
    # Paket yolları
    package_dir = get_package_share_directory('multiple_turtlebot3_sim')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # World dosyası
    world_path = os.path.join(package_dir, 'worlds', 'turtlebot3_world.world')

    # Gazebo model yolu
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join(package_dir, 'models')

    # Simülasyon başlat (gzserver + gzclient)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world_path}'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2'}.items(),
    )

    # Robot başlatıcı
    def spawn_robot(robot_name, x_pose, y_pose, z_pose, yaw, model_type="turtlebot3_burger"):
        model_path = os.path.join(package_dir, 'models', model_type, 'model.sdf')
        urdf_path = os.path.join(package_dir, 'urdf', f"{model_type}.urdf")

        # SDF içerik oku ve robot adını yerleştir
        with open(model_path, 'r') as f:
            sdf_content = f.read()
        sdf_content = sdf_content.replace('REPLACE_ROBOT_NAME', robot_name)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        return GroupAction([
            PushRosNamespace(robot_name),

            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',   robot_name,
                    '-string', sdf_content,
                    '-x',      str(x_pose),
                    '-y',      str(y_pose),
                    '-z',      str(z_pose),
                    '-R',     str(0),
                    '-P',     str(0),
                    '-Y',     str(yaw),
                ],
                output='screen',
            ),

            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    f'/{robot_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    f'/{robot_name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    f'/{robot_name}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                    f'/model/{robot_name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
                ],
                remappings=[
                    (f'/{robot_name}/odometry', f'/{robot_name}/odom'),
                    (f'/model/{robot_name}/tf', f'/{robot_name}/tf')
                ],
                output='screen'
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': Command(['xacro ', urdf_path]),
                    'frame_prefix': robot_name + '/',
                    'publish_frequency': 60.0,
                    'use_sim_time': True
                }],
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{robot_name}_world_to_odom_tf',
                arguments=[
                    str(x_pose), str(y_pose), str(z_pose),
                    str(qx), str(qy), str(qz), str(qw),
                    'world', f'{robot_name}/odom'
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),


        ])

    def yaw_to_quaternion(yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    # 3 robotu spawn ediyoruz
    robot1 = spawn_robot('robot1', -0.75, 0.75, 0.0, -0.75, model_type="turtlebot3_burger")
    robot2 = spawn_robot('robot2',  0.25, -0.25, 0.0, -2.38, model_type="turtlebot3_burger")
    robot3 = spawn_robot('robot3', 1.5, 0.5, 0.0, -3.14, model_type="turtlebot3_burger")

    main_robot = spawn_robot('main_robot', -1.7, 0.0, 0.0, 0.0, model_type="turtlebot3_burger")


    # tf_republisher (opsiyonel)
    tf_republishers = [
        Node(
            package='multiple_turtlebot3_sim',
            executable='tf_republisher',
            name='robot1_tf_republisher',
            arguments=['/robot1/tf'],
            output='screen',
        ),
        Node(
            package='multiple_turtlebot3_sim',
            executable='tf_republisher',
            name='robot2_tf_republisher',
            arguments=['/robot2/tf'],
            output='screen',
        ),
        Node(
            package='multiple_turtlebot3_sim',
            executable='tf_republisher',
            name='robot3_tf_republisher',
            arguments=['/robot3/tf'],
            output='screen',
        ),
        Node(
            package='multiple_turtlebot3_sim',
            executable='tf_republisher',
            name='main_robot_tf_republisher',
            arguments=['/main_robot/tf'],
            output='screen',
        ),
    ]


    # Launch Description
    ld = LaunchDescription()
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot1)
    ld.add_action(robot2)
    ld.add_action(robot3)
    ld.add_action(main_robot)
    for republisher in tf_republishers:
        ld.add_action(republisher)

    return ld
