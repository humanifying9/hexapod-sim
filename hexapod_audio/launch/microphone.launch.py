"""Launch file for the hexapod microphone node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    microphone_node = Node(
        package='hexapod_audio',
        executable='microphone_node',
        name='hexapod_microphone',
        output='screen',
        parameters=[
            {'sample_rate': 16000},
            {'channels': 1},
            {'frame_size': 512},
            {'use_hardware': False},
        ],
    )

    return LaunchDescription([microphone_node])
