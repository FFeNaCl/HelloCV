from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='referee_pkg',
            executable='hit_arror_server',
            name='hit_arror_server',
            output='screen'
        ),
        Node(
            package='referee_pkg',
            executable='fake_object_publisher',
            name='fake_object_publisher',
            output='screen'
        ),
        Node(
            package='referee_pkg',
            executable='hit_arror_client',
            name='hit_arror_client',
            output='screen'
        ),
    ])
