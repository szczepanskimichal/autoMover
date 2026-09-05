from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_path = get_package_share_directory(
        'automower_description'
    )

    xacro_file = os.path.join(
        package_path,
        'urdf',
        'automower.urdf.xacro'
    )

    rviz_config_file = os.path.join(
        package_path,
        'rviz',
        'automower.rviz'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    use_rviz = LaunchConfiguration('use_rviz')

    # Keep the learning path simple: one launch file starts description,
    # control, and optionally RViz for local inspection.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        Node(
            package='automower_control',
            executable='automower_drive_node'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz)
        ),
    ])