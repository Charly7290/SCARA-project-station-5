# Digital twin: real robot (via /scara_joints) + ghost of the IK target, in RViz
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('scara_description')
    twin = get_package_share_directory('scara_twin')
    params = os.path.join(twin, 'config', 'twin_params.yaml')
    ghost = LaunchConfiguration('ghost')

    return LaunchDescription([
        DeclareLaunchArgument('ghost', default_value='true',
                              description='Show the transparent ghost of the IK target'),

        # URDF + robot_state_publisher + RViz, without the sliders
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, 'launch', 'display.launch.py')),
            launch_arguments={'gui': 'false',
                              'rviz_config': os.path.join(twin, 'rviz', 'twin.rviz')}.items()),

        # /scara_joints -> /joint_states (+ vacuum marker)
        Node(package='scara_twin', executable='joint_bridge', output='screen',
             parameters=[params]),

        # Ghost: /scara_telemetry -> /ghost/joint_states -> ghost/* frames
        Node(package='scara_twin', executable='target_ghost', output='screen',
             parameters=[params], condition=IfCondition(ghost)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='ghost', condition=IfCondition(ghost),
             parameters=[{'robot_description': ParameterValue(
                              Command(['xacro ', os.path.join(desc, 'urdf', 'scara.urdf.xacro')]),
                              value_type=str),
                          'frame_prefix': 'ghost/'}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='ghost_base_tf',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'ghost/base_link'],
             condition=IfCondition(ghost)),
    ])
