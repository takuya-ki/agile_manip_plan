"""Dynamic replanning demo launch.

Starts the ``dynamic_replan_planner`` node alongside
robot_state_publisher and RViz. A moving box obstacle (driven by the
node's internal sinusoidal oscillator by default) is attached to each
cuMotion goal, and the latest trajectory is re-published to RViz
every replan tick so the changing plan is visible.
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

    config_file = os.path.join(examples_dir, 'config', 'dynamic_replan.yaml')
    urdf_xacro = os.path.join(
        desc_dir, 'urdf', 'iiwa14_robotiq_2f_140.urdf.xacro')
    rviz_file = os.path.join(desc_dir, 'rviz', 'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=config_file,
            description='Path to dynamic replan config YAML'),

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
            executable='dynamic_replan_planner',
            name='dynamic_replan_planner',
            output='screen',
            parameters=[LaunchConfiguration('config')]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file]),
    ])
