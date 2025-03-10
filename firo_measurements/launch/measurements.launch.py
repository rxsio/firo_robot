from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='firo_measurements',
            executable='measurements_subscriber',
            name='measurements_subscriber',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'topic_name': '/topic_name'}
            ]
        )
    ])