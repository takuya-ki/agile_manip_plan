"""Robot model visualization launch.

Brings up ``robot_state_publisher`` with the iiwa14 + Robotiq 2F-140
URDF and opens RViz with the display configuration from this package.
Use ``use_gui:=false`` to switch from the interactive joint slider GUI
(handy for manual inspection) to a headless ``joint_state_publisher``.
"""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('agile_manip_description')

    urdf_xacro = os.path.join(pkg_dir, 'urdf', 'iiwa14_robotiq_2f_140.urdf.xacro')
    # Demo 01 is pure robot visualization - use the minimal rviz config
    # so grasp/pipeline marker topics are not subscribed.
    rviz_file = os.path.join(pkg_dir, 'rviz', 'robot_model.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=launch.conditions.IfCondition(
                LaunchConfiguration('use_gui'))),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=launch.conditions.UnlessCondition(
                LaunchConfiguration('use_gui'))),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'),
    ])
