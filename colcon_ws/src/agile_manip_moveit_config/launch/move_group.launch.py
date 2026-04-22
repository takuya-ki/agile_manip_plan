"""Launch move_group + cuMotion planner action server for iiwa14 + Robotiq 2F-140.

Brings up:
- ``robot_state_publisher`` (iiwa14 + Robotiq 2F-140 URDF)
- ``joint_state_publisher`` (publishes zero joint state so move_group has a
  starting configuration; replace with a real controller when hardware is wired)
- ``move_group`` node (MoveIt 2, with ``isaac_ros_cumotion_moveit/CumotionPlanner``
  as the sole planning plugin)
- ``cumotion_planner_node`` (Isaac ROS cuMotion Python action server serving
  ``cumotion/move_group``)
- ``rviz2`` for visualization (disable with ``launch_rviz:=false``)
"""

import os
import tempfile

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r') as handle:
        return yaml.safe_load(handle) or {}


def _render_urdf(xacro_path, output_name, mappings=None):
    """Render the xacro into a plain URDF file on disk.

    cuMotion's ``get_robot_config`` passes the URDF path to cuRobo, which
    parses it with ``yourdfpy``; xacro directives are not understood, so we
    expand them up-front.
    """
    doc = xacro.process_file(xacro_path, mappings=mappings or {})
    urdf_xml = doc.toxml()
    temp_dir = os.path.join(tempfile.gettempdir(), 'agile_manip_moveit')
    os.makedirs(temp_dir, exist_ok=True)
    urdf_path = os.path.join(temp_dir, output_name)
    with open(urdf_path, 'w') as handle:
        handle.write(urdf_xml)
    return urdf_path, urdf_xml


def launch_setup(context, *args, **kwargs):
    desc_dir = get_package_share_directory('agile_manip_description')
    moveit_dir = get_package_share_directory('agile_manip_moveit_config')

    xacro_path = os.path.join(
        desc_dir, 'urdf', 'iiwa14_robotiq_2f_140.urdf.xacro')
    srdf_file = os.path.join(moveit_dir, 'config', 'iiwa14_robotiq_2f_140.srdf')
    xrdf_file = os.path.join(moveit_dir, 'config', 'iiwa14_robotiq_2f_140.xrdf')
    kinematics_file = os.path.join(moveit_dir, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(moveit_dir, 'config', 'joint_limits.yaml')
    controllers_file = os.path.join(moveit_dir, 'config', 'moveit_controllers.yaml')
    cumotion_planning_file = os.path.join(
        moveit_dir, 'config', 'isaac_ros_cumotion_planning.yaml')
    rviz_file = os.path.join(desc_dir, 'rviz', 'display.rviz')

    # Articulated Robotiq 2F-140 URDF (with finger_joint + mimic joints)
    # used by robot_state_publisher / RViz / MoveIt so the gripper can
    # visually open and close via /joint_states. cuMotion planning
    # receives the fixed-mesh variant because cuRobo fails when it has
    # to lock mimic joints.
    _, urdf_xml = _render_urdf(
        xacro_path, 'iiwa14_robotiq_2f_140.urdf',
        mappings={'articulated_gripper': 'true'})
    urdf_planning_path, _ = _render_urdf(
        xacro_path, 'iiwa14_robotiq_2f_140_planning.urdf',
        mappings={'articulated_gripper': 'false'})
    with open(srdf_file, 'r') as srdf_handle:
        srdf_xml = srdf_handle.read()

    robot_description = {'robot_description': urdf_xml}
    robot_description_semantic = {'robot_description_semantic': srdf_xml}
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml(kinematics_file),
    }
    robot_description_planning = {
        'robot_description_planning': _load_yaml(joint_limits_file),
    }

    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        _load_yaml(controllers_file),
        _load_yaml(cumotion_planning_file),
        {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        },
    ]

    cumotion_params = [
        robot_description,
        robot_description_semantic,
        {
            'urdf_path': urdf_planning_path,
            'yml_file_path': xrdf_file,
            'tool_frame': 'grasp_frame',
            'joint_states_topic': '/joint_states',
            'add_ground_plane': False,
            'read_esdf_world': False,
            'override_moveit_scaling_factors': False,
        },
    ]

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]),
        # joint_state_publisher is intentionally omitted: the demo
        # clients (cumotion_client, grasp_and_motion_planner,
        # obstacle_aware_grasp_and_motion_planner) publish their own
        # /joint_states (HOME at startup, then trajectory waypoints).
        # Running a second publisher that emits zeros here would
        # conflict with the client and cause the robot to flicker
        # between the zero pose and the client's trajectory in RViz.
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=move_group_params),
        Node(
            package='isaac_ros_cumotion',
            executable='cumotion_planner_node',
            name='cumotion_planner',
            output='screen',
            parameters=cumotion_params),
        Node(
            package='isaac_ros_cumotion',
            executable='static_planning_scene',
            name='static_planning_scene',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz alongside move_group'),
        OpaqueFunction(function=launch_setup),
    ])
