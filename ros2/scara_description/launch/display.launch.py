# Shows the SCARA model in RViz with sliders for each joint (no robot needed)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('scara_description')
    xacro_file = os.path.join(pkg, 'urdf', 'scara.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'scara.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Joint sliders (false when the real robot publishes /joint_states)'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_file,
                              description='RViz config (scara_twin uses its own with the ghost)'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),

        Node(package='rviz2', executable='rviz2', arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
