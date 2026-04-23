"""Benchmark harness launch.

Runs the ``benchmark_harness`` node headless against the same GraspGen
+ cuMotion backends used by the other demos. No RViz is started --
the goal is to produce per-iteration metrics in a CSV that can be
inspected or compared against a different ``selection_mode``.

For CLI overrides of individual parameters, prefer
``ros2 run agile_manip_examples benchmark_harness --ros-args ...``
so ``-p key:=value`` works directly; this launch file just wraps the
default YAML for convenience.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    examples_dir = get_package_share_directory('agile_manip_examples')
    config_file = os.path.join(examples_dir, 'config', 'benchmark.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=config_file,
            description='Path to benchmark harness config YAML'),

        Node(
            package='agile_manip_examples',
            executable='benchmark_harness',
            name='benchmark_harness',
            output='screen',
            parameters=[LaunchConfiguration('config')]),
    ])
