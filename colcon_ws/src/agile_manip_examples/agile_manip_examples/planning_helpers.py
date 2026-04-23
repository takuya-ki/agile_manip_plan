"""Shared helpers for grasp-and-motion planner nodes.

These functions factor out logic that was duplicated across
``grasp_and_motion_planner`` and ``obstacle_aware_grasp_and_motion_planner``.
They are plain functions (not a base class) to avoid perturbing the
existing node initialisation order; callers pass in whatever local
state they need.
"""

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from agile_manip_examples.cumotion_utils import (
    grasp_frame_xyz,
    width_to_finger_joint,
)


def order_grasp_candidates(grasp_candidates, selection_mode, selected_grasp_index):
    """Return ``(original_index, candidate)`` pairs in selection order.

    ``selection_mode == 'highest_confidence'`` puts scored grasps first,
    sorted by descending confidence, with unscored grasps appended at
    the end. Any other value falls back to rotating the list so that
    ``selected_grasp_index`` comes first, matching the prior behaviour.
    """
    indexed = list(enumerate(grasp_candidates))
    if selection_mode == 'highest_confidence':
        scored = [(i, c) for i, c in indexed if c.confidence is not None]
        unscored = [(i, c) for i, c in indexed if c.confidence is None]
        scored.sort(key=lambda entry: entry[1].confidence, reverse=True)
        return scored + unscored

    if not grasp_candidates:
        return []
    start_index = min(
        max(0, int(selected_grasp_index)),
        len(grasp_candidates) - 1,
    )
    return indexed[start_index:] + indexed[:start_index]


def resolve_gripper_value(gripper_width_m, gripper_finger_joint_target,
                         max_width_m=None):
    """Resolve the gripper target joint (rad) from the two alternative params.

    ``gripper_width_m >= 0`` takes precedence and is converted via the
    linear stroke model; otherwise the explicit finger-joint target is
    returned verbatim. ``max_width_m`` is forwarded to
    :func:`width_to_finger_joint` for non-2F-140 variants.
    """
    width = float(gripper_width_m)
    if width >= 0.0:
        if max_width_m is None:
            return width_to_finger_joint(width)
        return width_to_finger_joint(width, max_width_m=max_width_m)
    return float(gripper_finger_joint_target)


def build_trajectory_path_marker(trajectory, world_frame, stamp,
                                 namespace='trajectory_path', marker_id=1,
                                 rgba=(0.0, 0.6, 1.0, 0.8), line_width=0.005):
    """Build a LINE_STRIP marker tracing grasp_frame through ``trajectory``.

    Used by both planners to visualise the planned path in RViz. The
    geometry is derived from forward kinematics on each waypoint, so
    trajectories replayed with different gripper states produce the
    same line.
    """
    marker = Marker()
    marker.header.frame_id = world_frame
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = float(line_width)
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
    for joints in trajectory:
        x, y, z = grasp_frame_xyz(joints)
        marker.points.append(Point(x=x, y=y, z=z))
    return marker


def goal_residual_m(final_joints, selected_pose):
    """Euclidean distance between the trajectory end and the selected grasp.

    ``final_joints`` are the last waypoint's joint angles (iiwa-first
    ordering as returned by :func:`expand_robot_trajectory`). Returns
    the distance in metres from the FK-projected grasp_frame to the
    selected pose's position.
    """
    bx, by, bz = grasp_frame_xyz(final_joints)
    gx = selected_pose.position.x
    gy = selected_pose.position.y
    gz = selected_pose.position.z
    return ((bx - gx) ** 2 + (by - gy) ** 2 + (bz - gz) ** 2) ** 0.5


def log_goal_residual(logger, trajectory, selected_pose, tolerance_m=0.02):
    """Log whether the trajectory end matches the selected grasp pose."""
    if not trajectory or selected_pose is None:
        return
    residual = goal_residual_m(trajectory[-1], selected_pose)
    bx, by, bz = grasp_frame_xyz(trajectory[-1])
    gx = selected_pose.position.x
    gy = selected_pose.position.y
    gz = selected_pose.position.z
    if residual < float(tolerance_m):
        logger.info(
            f'Reached selected grasp pose (gripper-base residual '
            f'{residual*1000:.1f} mm).')
    else:
        logger.warn(
            f'Trajectory end is {residual*1000:.1f} mm away from the '
            f'selected grasp pose (gripper_base=({bx:.3f},{by:.3f},'
            f'{bz:.3f}), grasp=({gx:.3f},{gy:.3f},{gz:.3f})).')
