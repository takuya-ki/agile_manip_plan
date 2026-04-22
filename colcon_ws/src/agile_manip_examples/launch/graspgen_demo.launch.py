"""GraspGen demo launch.

Launches the local RViz-side client that calls the external
``graspgen_ros`` ``/generate_grasp`` service and republishes the
resulting grasp TFs as ``/graspgen/grasp_poses``. Start the actual
GraspGen server separately before using this launch file.
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

    config_file = os.path.join(examples_dir, 'config', 'graspgen_antipodal.yaml')
    urdf_xacro = os.path.join(desc_dir, 'urdf', 'iiwa14_robotiq_2f_140.urdf.xacro')
    rviz_file = os.path.join(desc_dir, 'rviz', 'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=config_file,
            description='Path to GraspGen config YAML'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description':
                    ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)
            }]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'),

        Node(
            package='agile_manip_examples',
            executable='graspgen_client',
            name='graspgen_client',
            output='screen',
            parameters=[LaunchConfiguration('config')]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file]),
    ])
