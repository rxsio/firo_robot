from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    telemetry_yaml = os.path.join(
        get_package_share_directory('rxsio_telemetry'),
        'config',
        'telemetry.yaml'
    )
    return LaunchDescription([
        Node(
            package='rxsio_telemetry',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'telemetry_yaml': telemetry_yaml}
            ]
        )
    ])