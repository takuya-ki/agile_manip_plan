"""Integrated grasp-and-motion demo launch.

Runs the local client that calls external GraspGen and cuMotion
backends, then republishes the selected grasp and planned trajectory
for RViz visualization. Start both backend stacks separately before
using this launch file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    examples_dir = get_package_share_directory('agile_manip_examples')
    desc_dir = get_package_share_directory('agile_manip_description')
    config_file = os.path.join(examples_dir, 'config', 'grasp_and_motion.yaml')
    urdf_xacro = os.path.join(
        desc_dir, 'urdf', 'iiwa14_robotiq_2f_140.urdf.xacro')
    rviz_file = os.path.join(desc_dir, 'rviz', 'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=config_file,
            description='Path to integrated GraspGen + cuMotion config YAML'),

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
            executable='grasp_and_motion_planner',
            name='grasp_and_motion_planner',
            output='screen',
            parameters=[LaunchConfiguration('config')]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file]),
    ])
