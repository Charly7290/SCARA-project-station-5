from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Gamepad driver, publishes /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'autorepeat_rate': 20.0,  # keeps publishing while nothing changes
                'deadzone': 0.0,  # handled in controller_teleop
            }],
        ),
        Node(
            package='scara_teleop',
            executable='controller_teleop',
            name='scara_controller_teleop',
            output='screen',
            # Set invert_* to True if a joint moves the wrong way (then colcon build)
            parameters=[{
                'j1_axis': 0,               # left stick X
                'j2_axis': 3,               # right stick X
                'invert_left_stick': False, # J1 direction
                'invert_right_stick': False,# J2 direction
            }],
        ),
    ])
