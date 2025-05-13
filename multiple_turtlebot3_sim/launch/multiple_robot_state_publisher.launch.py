import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ns          = LaunchConfiguration("namespace")
    frame_pref  = LaunchConfiguration("frame_prefix")
    use_sim     = LaunchConfiguration("use_sim_time")

    urdf = os.path.join(
        get_package_share_directory("multiple_turtlebot3_sim"),
        "urdf", "turtlebot3_burger.urdf"
    )
    with open(urdf, "r") as f: robot_desc = f.read()

    print("test")

    return LaunchDescription([
        DeclareLaunchArgument("namespace"),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim,
                "robot_description": robot_desc,
                "frame_prefix": frame_pref
            }],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
    ])
