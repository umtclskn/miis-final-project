from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_names = ['robot1', 'robot2', 'robot3']
    nodes = []

    for name in robot_names:
        nodes.append(
            Node(
                package='robot_path_tracker',
                executable='path_tracker',
                name=f'{name}_path_tracker',
                namespace=name,
                parameters=[
                    {'robot_name': name},
                    {'obstacle_avoidance': True},
                    {'robot_collision_avoidance': True}  # Çarpışma önleme parametresi eklendi
                ],
                output='screen'
            )
        )

    return LaunchDescription(nodes)