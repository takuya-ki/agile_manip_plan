"""Shared helpers for grasp-and-motion planner nodes.

These functions factor out logic that was duplicated across
``grasp_and_motion_planner`` and ``obstacle_aware_grasp_and_motion_planner``.
They are plain functions (not a base class) to avoid perturbing the
existing node initialisation order; callers pass in whatever local
state they need.
"""

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from agile_manip_examples.cumotion_utils import (
    grasp_frame_xyz,
    width_to_finger_joint,
)


# Sweet-spot distance (metres) from the iiwa14 base at which the
# reachability sub-score peaks. The iiwa14 R820 has 820 mm reach, so
# placing grasps ~0.5 m out leaves comfortable margin on both ends.
REACH_SWEET_SPOT_M = 0.5
REACH_FALLOFF_M = 0.35  # distance from sweet spot that drives the score to 0


def _reachability_score(pose):
    """Return 0..1 score: 1 at the sweet spot, 0 beyond the falloff.

    Uses a cosine window so the score decays smoothly rather than with
    a sharp knee -- helps break ties between grasps that are all
    close-but-not-exactly at REACH_SWEET_SPOT_M.
    """
    dx = pose.position.x
    dy = pose.position.y
    dz = pose.position.z
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    delta = abs(dist - REACH_SWEET_SPOT_M)
    if delta >= REACH_FALLOFF_M:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * delta / REACH_FALLOFF_M))


def score_grasp_candidate(candidate,
                          weight_confidence=0.7,
                          weight_reach=0.3):
    """Combined score for multi-criteria grasp ranking.

    Returns a weighted sum of the confidence and reachability
    sub-scores. Both are normalised to [0, 1]; weights are supplied
    by the caller (typically from ROS parameters) and should sum to
    1.0 for an interpretable score -- but they are not enforced so
    callers can experiment.

    Only task-agnostic geometric criteria are included here:
    ``confidence`` is GraspGen's own success-probability estimate,
    and ``reach`` favours grasps inside the iiwa14's comfortable
    reach envelope (a property of the robot, not the task).
    Task-specific heuristics (top-down preference, specific approach
    directions, clearance to obstacles) should be added by composing
    this helper with a user-provided scorer rather than baking them
    in here.

    ``candidate.confidence`` may be ``None`` for grasps that were not
    accompanied by a GraspGen score file; such grasps contribute 0 to
    the confidence term but are still ranked on reach.
    """
    confidence = (
        float(candidate.confidence) if candidate.confidence is not None else 0.0)
    reach = _reachability_score(candidate.pose)
    return (
        float(weight_confidence) * confidence
        + float(weight_reach) * reach
    )


def order_grasp_candidates(grasp_candidates, selection_mode, selected_grasp_index,
                           multi_criteria_weights=None):
    """Return ``(original_index, candidate)`` pairs in selection order.

    Supported ``selection_mode`` values:

    - ``'highest_confidence'`` (default in existing configs): scored
      grasps sorted by descending confidence, then unscored grasps.
    - ``'multi_criteria'``: scored by :func:`score_grasp_candidate`
      which combines GraspGen confidence with a reachability term.
      ``multi_criteria_weights`` is a dict ``{'confidence', 'reach'}``;
      defaults are used when omitted.
    - Anything else (``'manual'``): put ``selected_grasp_index`` first,
      then the remaining candidates in confidence-descending order so
      fallback is still meaningful if the chosen pose fails IK.
    """
    indexed = list(enumerate(grasp_candidates))
    if not grasp_candidates:
        return []

    if selection_mode == 'highest_confidence':
        scored = [(i, c) for i, c in indexed if c.confidence is not None]
        unscored = [(i, c) for i, c in indexed if c.confidence is None]
        scored.sort(key=lambda entry: entry[1].confidence, reverse=True)
        return scored + unscored

    if selection_mode == 'multi_criteria':
        weights = multi_criteria_weights or {}
        scored_entries = [
            (score_grasp_candidate(
                c,
                weight_confidence=weights.get('confidence', 0.7),
                weight_reach=weights.get('reach', 0.3),
            ), i, c)
            for i, c in indexed
        ]
        # Descending score; stable on original index so ties are
        # reproducible across runs.
        scored_entries.sort(key=lambda entry: (-entry[0], entry[1]))
        return [(i, c) for _, i, c in scored_entries]

    # manual mode: start from the user-picked index, then fall back in
    # confidence order rather than a meaningless numeric rotation.
    start_index = min(
        max(0, int(selected_grasp_index)),
        len(grasp_candidates) - 1,
    )
    head = indexed[start_index]
    rest = [entry for entry in indexed if entry[0] != start_index]
    rest.sort(
        key=lambda entry: (
            entry[1].confidence if entry[1].confidence is not None else -1.0),
        reverse=True,
    )
    return [head] + rest


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


def trajectory_jerk_metrics(robot_trajectory):
    """Compute RMS and max jerk across a ``RobotTrajectory``.

    Jerk is the third time-derivative of joint position (rad/s^3).
    It captures how *smoothly* the arm would move through the plan
    -- low jerk = comfortable / easy-on-hardware, high jerk = sharp
    transitions that shake the robot. Trajectory length alone cannot
    express this.

    If the incoming trajectory's points already carry ``accelerations``
    entries (MoveIt / cuMotion usually populate these), we numerically
    differentiate them against ``time_from_start`` to get jerk.
    Otherwise we fall back to differentiating positions twice. In both
    cases we only look at the 7 arm joints we plan for; the gripper
    column appended by :func:`expand_robot_trajectory` is ignored.

    Returns ``(rms, max_abs)`` in rad/s^3 (or ``(0.0, 0.0)`` if the
    trajectory is too short to differentiate).
    """
    points = list(robot_trajectory.joint_trajectory.points)
    if len(points) < 4:
        return 0.0, 0.0

    def _to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    times = [_to_seconds(p.time_from_start) for p in points]
    # If cuMotion / OMPL return all-zero timestamps (seen occasionally
    # on degenerate plans), manufacture a uniform 100 ms spacing so
    # the metric is still defined rather than NaN.
    if all(t == 0.0 for t in times):
        times = [0.1 * i for i in range(len(points))]
    # Guard against duplicated timestamps which would divide by zero.
    dts = [max(times[i + 1] - times[i], 1e-6) for i in range(len(times) - 1)]

    have_accel = all(len(p.accelerations) > 0 for p in points)
    if have_accel:
        # jerk[i] = (a[i+1] - a[i]) / dt[i]
        n_joints = len(points[0].accelerations)
        jerks = []
        for i in range(len(points) - 1):
            for j in range(n_joints):
                jerks.append(
                    (points[i + 1].accelerations[j]
                     - points[i].accelerations[j]) / dts[i])
    else:
        # Fall back: velocity -> acceleration -> jerk from positions.
        n_joints = len(points[0].positions)
        # Velocities at midpoints between waypoints.
        velocities = [
            [(points[i + 1].positions[j] - points[i].positions[j]) / dts[i]
             for j in range(n_joints)]
            for i in range(len(points) - 1)
        ]
        # Accelerations via central differences on velocities.
        accels = [
            [(velocities[i + 1][j] - velocities[i][j])
             / (0.5 * (dts[i] + dts[i + 1]))
             for j in range(n_joints)]
            for i in range(len(velocities) - 1)
        ]
        # Jerks = finite diff of accelerations.
        jerks = []
        for i in range(len(accels) - 1):
            step = 0.5 * (dts[i + 1] + dts[i + 2])
            for j in range(n_joints):
                jerks.append((accels[i + 1][j] - accels[i][j]) / step)

    if not jerks:
        return 0.0, 0.0
    rms = (sum(j * j for j in jerks) / len(jerks)) ** 0.5
    max_abs = max(abs(j) for j in jerks)
    return rms, max_abs


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
