"""Helpers for sending cuMotion MoveGroup goals and unpacking results."""

import math

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from agile_manip_examples.ik_utils import iiwa_grasp_frame_fk


ARM_JOINT_NAMES = (
    'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4',
    'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7',
)

# Robotiq 2F-140 is articulated via ``finger_joint`` + 5 mimic joints
# in the visualization URDF. cuMotion plans the arm only, but we still
# publish ``finger_joint`` in /joint_states so robot_state_publisher
# can compute the mimic transforms and RViz shows the gripper opening
# or closing as commanded. Range: 0.0 rad (fully open, ~140 mm stroke)
# to ~0.70 rad (fully closed).
GRIPPER_JOINT_NAME = 'finger_joint'
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.7

JOINT_NAMES = ARM_JOINT_NAMES + (GRIPPER_JOINT_NAME,)

HOME_ARM_JOINTS = [0.0, 0.0, 0.0, -math.pi / 2, 0.0, math.pi / 2, 0.0]
HOME_JOINTS = HOME_ARM_JOINTS + [GRIPPER_OPEN]


def width_to_finger_joint(width_m):
    """Convert a desired jaw opening (in metres) to finger_joint (rad).

    The Robotiq 2F-140 has a ~140 mm stroke, so width_m=0.14 ≈ fully
    open and width_m=0.0 ≈ fully closed. The mapping is approximately
    linear against the driving joint angle.
    """
    max_width = 0.140
    clamped = max(0.0, min(max_width, float(width_m)))
    # Inverse of approx. linear relation (open stroke ↔ joint angle):
    return GRIPPER_CLOSED * (1.0 - clamped / max_width)


def make_start_state(joint_names, joint_positions):
    """Create a MoveIt start state."""
    state = RobotState()
    state.joint_state = JointState()
    state.joint_state.name = list(joint_names)
    state.joint_state.position = list(joint_positions)
    return state


def make_joint_constraints(joint_names, joint_positions):
    """Build joint constraints for a MoveGroup goal."""
    constraints = Constraints()
    for joint_name, position in zip(joint_names, joint_positions):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = float(position)
        joint_constraint.tolerance_above = 1e-3
        joint_constraint.tolerance_below = 1e-3
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    return constraints


def make_pose_constraints(frame_id, end_effector_link, target_pose):
    """Build pose constraints for a MoveGroup goal."""
    constraints = Constraints()

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = frame_id
    position_constraint.link_name = end_effector_link
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [0.005]
    position_constraint.constraint_region.primitives.append(primitive)
    position_constraint.constraint_region.primitive_poses.append(target_pose)
    position_constraint.weight = 1.0
    constraints.position_constraints.append(position_constraint)

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = frame_id
    orientation_constraint.link_name = end_effector_link
    orientation_constraint.orientation = target_pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.05
    orientation_constraint.absolute_y_axis_tolerance = 0.05
    orientation_constraint.absolute_z_axis_tolerance = 0.05
    orientation_constraint.weight = 1.0
    constraints.orientation_constraints.append(orientation_constraint)

    return constraints


def build_joint_goal(
        group_name,
        pipeline_id,
        planner_id,
        start_joint_positions,
        goal_joint_positions,
        allowed_planning_time):
    """Create a joint-space MoveGroup goal for cuMotion."""
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = group_name
    goal_msg.request.pipeline_id = pipeline_id
    goal_msg.request.planner_id = planner_id
    goal_msg.request.num_planning_attempts = 1
    goal_msg.request.allowed_planning_time = float(allowed_planning_time)
    goal_msg.request.max_velocity_scaling_factor = 1.0
    goal_msg.request.max_acceleration_scaling_factor = 1.0
    goal_msg.request.start_state = make_start_state(
        ARM_JOINT_NAMES, start_joint_positions)
    goal_msg.request.goal_constraints.append(
        make_joint_constraints(ARM_JOINT_NAMES, goal_joint_positions))
    goal_msg.planning_options.plan_only = True
    return goal_msg


def build_pose_goal(
        group_name,
        pipeline_id,
        planner_id,
        end_effector_link,
        world_frame,
        start_joint_positions,
        target_pose,
        allowed_planning_time):
    """Create a pose goal for cuMotion."""
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = group_name
    goal_msg.request.pipeline_id = pipeline_id
    goal_msg.request.planner_id = planner_id
    goal_msg.request.num_planning_attempts = 1
    goal_msg.request.allowed_planning_time = float(allowed_planning_time)
    goal_msg.request.max_velocity_scaling_factor = 1.0
    goal_msg.request.max_acceleration_scaling_factor = 1.0
    goal_msg.request.start_state = make_start_state(
        ARM_JOINT_NAMES, start_joint_positions)
    goal_msg.request.goal_constraints.append(
        make_pose_constraints(world_frame, end_effector_link, target_pose))
    goal_msg.planning_options.plan_only = True
    return goal_msg


def expand_robot_trajectory(robot_trajectory, gripper_value=GRIPPER_OPEN):
    """Project a cuMotion trajectory onto ``JOINT_NAMES`` ordering.

    cuMotion only plans the 7 iiwa joints. ``gripper_value`` (rad) is
    appended so every waypoint has the full ``JOINT_NAMES`` layout and
    robot_state_publisher can compute TFs for the articulated gripper.
    """
    trajectory = []
    joint_names = list(robot_trajectory.joint_trajectory.joint_names)
    for point in robot_trajectory.joint_trajectory.points:
        point_map = {
            joint_name: position
            for joint_name, position in zip(joint_names, point.positions)
        }
        full_point = [point_map.get(joint_name, 0.0)
                      for joint_name in ARM_JOINT_NAMES]
        full_point.append(float(gripper_value))
        trajectory.append(full_point)
    return trajectory


def moveit_error_name(error_code_value):
    """Return a readable MoveIt error-code label."""
    code_map = {
        MoveItErrorCodes.SUCCESS: 'SUCCESS',
        MoveItErrorCodes.FAILURE: 'FAILURE',
        MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
        MoveItErrorCodes.INVALID_MOTION_PLAN: 'INVALID_MOTION_PLAN',
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
        MoveItErrorCodes.CONTROL_FAILED: 'CONTROL_FAILED',
        MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
        MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
        MoveItErrorCodes.PREEMPTED: 'PREEMPTED',
        MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
        MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            'START_STATE_VIOLATES_PATH_CONSTRAINTS',
        MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
        MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            'GOAL_VIOLATES_PATH_CONSTRAINTS',
        MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'GOAL_CONSTRAINTS_VIOLATED',
        MoveItErrorCodes.INVALID_GROUP_NAME: 'INVALID_GROUP_NAME',
        MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'INVALID_GOAL_CONSTRAINTS',
        MoveItErrorCodes.INVALID_ROBOT_STATE: 'INVALID_ROBOT_STATE',
        MoveItErrorCodes.INVALID_LINK_NAME: 'INVALID_LINK_NAME',
        MoveItErrorCodes.NO_IK_SOLUTION: 'NO_IK_SOLUTION',
    }
    return code_map.get(error_code_value, f'UNKNOWN({error_code_value})')


def grasp_frame_xyz(joints):
    """``grasp_frame`` position via iiwa14 FK.

    This is the link cuMotion is asked to place at a GraspGen pose, so
    both trajectory previews and residual checks against the selected
    grasp use this helper for consistency.
    """
    transform = iiwa_grasp_frame_fk(joints[:7])
    return float(transform[0, 3]), float(transform[1, 3]), float(transform[2, 3])
