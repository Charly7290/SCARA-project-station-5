from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scara_teleop',
            executable='keyboard_teleop',
            name='scara_keyboard_teleop',
            output='screen',
        ),
    ])
