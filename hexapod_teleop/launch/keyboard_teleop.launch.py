"""Launch file for WASD keyboard teleop. Run this in its own terminal -
it needs an interactive tty and captures raw keystrokes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    keyboard_teleop_node = Node(
        package='hexapod_teleop',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'linear_speed': 0.05},
            {'angular_speed': 0.4},
        ],
    )

    return LaunchDescription([keyboard_teleop_node])
