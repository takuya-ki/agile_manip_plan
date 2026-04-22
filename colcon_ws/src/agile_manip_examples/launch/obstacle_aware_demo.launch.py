"""Obstacle-aware GraspGen + cuMotion demo launch.

Starts the local client that calls the external GraspGen and cuMotion
backends. The client injects a static box obstacle into MoveIt's
planning scene before requesting a plan, so cuMotion routes the arm
around it and the resulting trajectory is visibly different from the
obstacle-free :mod:`grasp_and_motion_demo`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    examples_dir = get_package_share_directory('agile_manip_examples')
    desc_dir = get_package_share_directory('agile_manip_description')

    config_file = os.path.join(examples_dir, 'config', 'obstacle_aware.yaml')
    urdf_xacro = os.path.join(
        desc_dir, 'urdf', 'iiwa14_robotiq_2f_140.urdf.xacro')
    rviz_file = os.path.join(desc_dir, 'rviz', 'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=config_file,
            description='Path to obstacle-aware GraspGen + cuMotion config YAML'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_xacro]), value_type=str),
            }]),

        Node(
            package='agile_manip_examples',
            executable='obstacle_aware_grasp_and_motion_planner',
            name='obstacle_aware_grasp_and_motion_planner',
            output='screen',
            parameters=[LaunchConfiguration('config')]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file]),
    ])
