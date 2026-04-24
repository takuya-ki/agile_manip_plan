"""Unit tests for planning_helpers.

These tests stub the ROS-message and cuMotion-utils imports so the
pure-Python helpers can be exercised without a ROS install. The goal
is to guard the refactor that pulled these helpers out of the two
grasp_and_motion planner nodes.
"""

import sys
import types
from dataclasses import dataclass
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# Stub ROS / cuMotion dependencies so planning_helpers can import on host.
# ---------------------------------------------------------------------------

def _ensure_module(name):
    module = sys.modules.get(name)
    if module is None:
        module = types.ModuleType(name)
        sys.modules[name] = module
    return module


# geometry_msgs.msg.Point -- simple stand-in with xyz attributes.
geometry_msgs = _ensure_module('geometry_msgs')
geometry_msgs_msg = _ensure_module('geometry_msgs.msg')


class _StubPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _StubPose:
    def __init__(self):
        self.position = _StubPoint()
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


geometry_msgs_msg.Point = _StubPoint
geometry_msgs_msg.Pose = _StubPose
geometry_msgs.msg = geometry_msgs_msg


# visualization_msgs.msg.Marker -- only attributes the helper sets.
visualization_msgs = _ensure_module('visualization_msgs')
visualization_msgs_msg = _ensure_module('visualization_msgs.msg')


class _StubMarker:
    LINE_STRIP = 4
    ADD = 0

    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='', stamp=None)
        self.ns = ''
        self.id = 0
        self.type = 0
        self.action = 0
        self.scale = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.color = types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.points = []


visualization_msgs_msg.Marker = _StubMarker
visualization_msgs.msg = visualization_msgs_msg


# Stub agile_manip_examples.cumotion_utils to avoid pulling in moveit_msgs.
cumotion_stub = _ensure_module('agile_manip_examples.cumotion_utils')


def _stub_grasp_frame_xyz(joints):
    # Identity-ish projection that lets us write deterministic tests.
    return float(joints[0]), float(joints[1]), float(joints[2])


def _stub_width_to_finger_joint(width_m, max_width_m=0.140):
    clamped = max(0.0, min(float(max_width_m), float(width_m)))
    return 0.7 * (1.0 - clamped / float(max_width_m))


cumotion_stub.grasp_frame_xyz = _stub_grasp_frame_xyz
cumotion_stub.width_to_finger_joint = _stub_width_to_finger_joint


# Make the package importable from source.
SRC_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC_ROOT))

# Package shim: register agile_manip_examples so the real submodule imports
# of planning_helpers resolve without executing the package __init__.
agile_pkg = _ensure_module('agile_manip_examples')
agile_pkg.__path__ = [str(SRC_ROOT / 'agile_manip_examples')]

from agile_manip_examples import planning_helpers  # noqa: E402


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@dataclass
class FakeCandidate:
    confidence: float | None
    pose: object = None


def _pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    pose = _StubPose()
    pose.position = _StubPoint(x, y, z)
    pose.orientation = types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw)
    return pose


@pytest.fixture
def sample_candidates():
    # Poses land near the reach sweet spot so multi_criteria tests can
    # still discriminate on confidence; specific XYZ values don't affect
    # the highest_confidence / manual modes.
    return [
        FakeCandidate(confidence=0.10, pose=_pose(0.5, 0.0, 0.0)),
        FakeCandidate(confidence=None, pose=_pose(0.5, 0.0, 0.0)),
        FakeCandidate(confidence=0.80, pose=_pose(0.5, 0.0, 0.0)),
        FakeCandidate(confidence=0.50, pose=_pose(0.5, 0.0, 0.0)),
    ]


# ---------------------------------------------------------------------------
# order_grasp_candidates
# ---------------------------------------------------------------------------

def test_highest_confidence_puts_best_scored_first(sample_candidates):
    ordered = planning_helpers.order_grasp_candidates(
        sample_candidates, selection_mode='highest_confidence',
        selected_grasp_index=0)
    original_indices = [idx for idx, _ in ordered]
    # 2 (0.80), 3 (0.50), 0 (0.10), then 1 (unscored) last.
    assert original_indices == [2, 3, 0, 1]


def test_highest_confidence_preserves_pairing(sample_candidates):
    ordered = planning_helpers.order_grasp_candidates(
        sample_candidates, selection_mode='highest_confidence',
        selected_grasp_index=0)
    for original_idx, candidate in ordered:
        assert sample_candidates[original_idx] is candidate


def test_manual_starts_with_picked_index_and_falls_back_by_confidence(
        sample_candidates):
    # User picks index 2. If that grasp fails IK, the remaining grasps
    # should be tried in descending confidence: 3 (0.50), 0 (0.10),
    # 1 (None) -- not a mindless numeric rotation.
    ordered = planning_helpers.order_grasp_candidates(
        sample_candidates, selection_mode='manual',
        selected_grasp_index=2)
    assert [idx for idx, _ in ordered] == [2, 3, 0, 1]


def test_manual_picks_the_index_even_when_others_have_higher_confidence(
        sample_candidates):
    # selected_grasp_index=0 puts the low-confidence grasp at the head
    # exactly because the caller asked for it; the fallback is then
    # sorted by confidence so 2 (0.80) comes next, not 1 (None).
    ordered = planning_helpers.order_grasp_candidates(
        sample_candidates, selection_mode='manual',
        selected_grasp_index=0)
    assert [idx for idx, _ in ordered] == [0, 2, 3, 1]


def test_manual_index_clamped_to_range(sample_candidates):
    ordered = planning_helpers.order_grasp_candidates(
        sample_candidates, selection_mode='manual',
        selected_grasp_index=99)
    assert ordered[0][0] == len(sample_candidates) - 1


def test_manual_index_negative_clamped_to_zero(sample_candidates):
    ordered = planning_helpers.order_grasp_candidates(
        sample_candidates, selection_mode='manual',
        selected_grasp_index=-5)
    assert ordered[0][0] == 0


def test_empty_list_returns_empty():
    assert planning_helpers.order_grasp_candidates(
        [], 'highest_confidence', 0) == []
    assert planning_helpers.order_grasp_candidates(
        [], 'manual', 0) == []
    assert planning_helpers.order_grasp_candidates(
        [], 'multi_criteria', 0) == []


# ---------------------------------------------------------------------------
# multi_criteria mode
# ---------------------------------------------------------------------------

def test_multi_criteria_prefers_higher_combined_score():
    # Two grasps with identical pose -> confidence decides the order.
    candidates = [
        FakeCandidate(confidence=0.2, pose=_pose(0.5, 0.0, 0.0)),
        FakeCandidate(confidence=0.9, pose=_pose(0.5, 0.0, 0.0)),
    ]
    ordered = planning_helpers.order_grasp_candidates(
        candidates, 'multi_criteria', 0)
    assert [i for i, _ in ordered] == [1, 0]


def test_multi_criteria_uses_reachability_to_break_confidence_ties():
    # Same confidence but pose 1 sits at the sweet spot, pose 0 is far.
    candidates = [
        FakeCandidate(confidence=0.5, pose=_pose(2.0, 0.0, 0.0)),  # unreachable
        FakeCandidate(confidence=0.5, pose=_pose(0.5, 0.0, 0.0)),  # sweet spot
    ]
    ordered = planning_helpers.order_grasp_candidates(
        candidates, 'multi_criteria', 0)
    assert [i for i, _ in ordered] == [1, 0]


def test_multi_criteria_weights_can_disable_confidence():
    # With confidence weight 0, the lower-confidence but better-placed
    # grasp wins on reach alone.
    candidates = [
        FakeCandidate(confidence=0.9, pose=_pose(2.0, 0.0, 0.0)),
        FakeCandidate(confidence=0.1, pose=_pose(0.5, 0.0, 0.0)),
    ]
    ordered = planning_helpers.order_grasp_candidates(
        candidates, 'multi_criteria', 0,
        multi_criteria_weights={'confidence': 0.0, 'reach': 1.0})
    assert [i for i, _ in ordered] == [1, 0]


def test_multi_criteria_handles_none_confidence():
    # None confidence is treated as 0 so the scored candidate still
    # wins on the confidence term.
    candidates = [
        FakeCandidate(confidence=None, pose=_pose(0.5, 0.0, 0.0)),
        FakeCandidate(confidence=0.3, pose=_pose(0.5, 0.0, 0.0)),
    ]
    ordered = planning_helpers.order_grasp_candidates(
        candidates, 'multi_criteria', 0)
    assert [i for i, _ in ordered] == [1, 0]


# ---------------------------------------------------------------------------
# resolve_gripper_value
# ---------------------------------------------------------------------------

def test_width_takes_precedence_when_non_negative():
    # width=0 -> fully closed in the linear model ~ GRIPPER_CLOSED (0.7).
    assert planning_helpers.resolve_gripper_value(
        0.0, gripper_finger_joint_target=0.3) == pytest.approx(0.7)


def test_negative_width_falls_back_to_joint_target():
    assert planning_helpers.resolve_gripper_value(
        -1.0, gripper_finger_joint_target=0.25) == pytest.approx(0.25)


def test_width_full_open_maps_to_zero():
    assert planning_helpers.resolve_gripper_value(
        0.14, gripper_finger_joint_target=0.3) == pytest.approx(0.0, abs=1e-6)


def test_width_custom_max():
    # With a 0.08 m stroke, width=0.04 sits mid-way so result ~ 0.35 rad.
    value = planning_helpers.resolve_gripper_value(
        0.04, gripper_finger_joint_target=0.0, max_width_m=0.08)
    assert value == pytest.approx(0.35, abs=1e-6)


# ---------------------------------------------------------------------------
# goal_residual_m
# ---------------------------------------------------------------------------

def test_goal_residual_zero_when_on_target():
    pose = _StubPose()
    pose.position = _StubPoint(1.0, 2.0, 3.0)
    assert planning_helpers.goal_residual_m(
        [1.0, 2.0, 3.0, 0, 0, 0, 0], pose) == pytest.approx(0.0)


def test_goal_residual_non_zero_when_off_target():
    pose = _StubPose()
    pose.position = _StubPoint(0.0, 0.0, 0.0)
    residual = planning_helpers.goal_residual_m(
        [3.0, 4.0, 0.0, 0, 0, 0, 0], pose)
    assert residual == pytest.approx(5.0)  # 3-4-5 triangle


# ---------------------------------------------------------------------------
# build_trajectory_path_marker
# ---------------------------------------------------------------------------

def test_path_marker_has_one_point_per_waypoint():
    trajectory = [[i * 0.1, 0.0, 0.0, 0, 0, 0, 0] for i in range(5)]
    marker = planning_helpers.build_trajectory_path_marker(
        trajectory, world_frame='world', stamp=None)
    assert len(marker.points) == 5
    assert marker.header.frame_id == 'world'
    assert marker.type == _StubMarker.LINE_STRIP
    assert marker.points[0].x == pytest.approx(0.0)
    assert marker.points[4].x == pytest.approx(0.4)


def test_path_marker_accepts_empty_trajectory():
    marker = planning_helpers.build_trajectory_path_marker(
        [], world_frame='world', stamp=None)
    assert marker.points == []


# ---------------------------------------------------------------------------
# log_goal_residual
# ---------------------------------------------------------------------------

class _RecordingLogger:
    def __init__(self):
        self.info_msgs = []
        self.warn_msgs = []

    def info(self, msg):
        self.info_msgs.append(msg)

    def warn(self, msg):
        self.warn_msgs.append(msg)


def test_log_goal_residual_logs_info_when_within_tolerance():
    logger = _RecordingLogger()
    pose = _StubPose()
    pose.position = _StubPoint(0.0, 0.0, 0.0)
    planning_helpers.log_goal_residual(
        logger, [[0.001, 0.0, 0.0, 0, 0, 0, 0]], pose, tolerance_m=0.02)
    assert len(logger.info_msgs) == 1
    assert logger.warn_msgs == []


def test_log_goal_residual_logs_warn_when_outside_tolerance():
    logger = _RecordingLogger()
    pose = _StubPose()
    pose.position = _StubPoint(0.0, 0.0, 0.0)
    planning_helpers.log_goal_residual(
        logger, [[0.1, 0.0, 0.0, 0, 0, 0, 0]], pose, tolerance_m=0.02)
    assert logger.info_msgs == []
    assert len(logger.warn_msgs) == 1


def test_log_goal_residual_noop_when_no_trajectory():
    logger = _RecordingLogger()
    planning_helpers.log_goal_residual(logger, [], _StubPose())
    assert logger.info_msgs == []
    assert logger.warn_msgs == []


def test_log_goal_residual_noop_when_no_pose():
    logger = _RecordingLogger()
    planning_helpers.log_goal_residual(logger, [[0, 0, 0, 0, 0, 0, 0]], None)
    assert logger.info_msgs == []
    assert logger.warn_msgs == []


# ---------------------------------------------------------------------------
# trajectory_jerk_metrics
# ---------------------------------------------------------------------------

def _fake_trajectory(points_positions, points_accelerations=None, dt=0.1):
    """Build a stand-in RobotTrajectory-like object."""
    points = []
    for i, pos in enumerate(points_positions):
        stamp = types.SimpleNamespace(sec=int(i * dt), nanosec=int((i * dt % 1) * 1e9))
        pt = types.SimpleNamespace(
            positions=list(pos),
            accelerations=list(points_accelerations[i])
                           if points_accelerations is not None else [],
            time_from_start=stamp,
        )
        points.append(pt)
    jt = types.SimpleNamespace(joint_trajectory=types.SimpleNamespace(points=points))
    return jt


def test_jerk_zero_for_constant_acceleration():
    # 7-joint trajectory with acceleration vector constant in time ->
    # zero jerk.
    n = 6
    accels = [[0.5] * 7 for _ in range(n)]
    positions = [[0.0] * 7 for _ in range(n)]
    traj = _fake_trajectory(positions, accels)
    rms, mx = planning_helpers.trajectory_jerk_metrics(traj)
    assert rms == pytest.approx(0.0, abs=1e-9)
    assert mx == pytest.approx(0.0, abs=1e-9)


def test_jerk_nonzero_when_acceleration_changes():
    # Step in acceleration on one joint; expect finite non-zero jerk.
    accels = [[0.0] * 7, [0.0] * 7, [1.0] + [0.0] * 6,
              [1.0] + [0.0] * 6, [0.0] * 7, [0.0] * 7]
    positions = [[0.0] * 7 for _ in range(len(accels))]
    traj = _fake_trajectory(positions, accels, dt=0.1)
    rms, mx = planning_helpers.trajectory_jerk_metrics(traj)
    assert mx > 0.0
    assert rms > 0.0


def test_jerk_falls_back_to_positions_when_accel_missing():
    # Cubic position profile on one joint -> jerk is the 3rd derivative
    # coefficient (constant), magnitude 6 rad/s^3 for position = t^3.
    dt = 0.1
    positions = [[((i * dt) ** 3)] + [0.0] * 6 for i in range(8)]
    traj = _fake_trajectory(positions, points_accelerations=None, dt=dt)
    rms, mx = planning_helpers.trajectory_jerk_metrics(traj)
    # Numerical 3rd-diff on t^3 gives ~6.0 once the discretisation
    # settles; accept a wide tolerance because end-effects inflate max.
    assert mx == pytest.approx(6.0, rel=0.2)


def test_jerk_zero_for_short_trajectory():
    traj = _fake_trajectory([[0.0] * 7, [0.1] * 7, [0.2] * 7])
    rms, mx = planning_helpers.trajectory_jerk_metrics(traj)
    assert rms == 0.0 and mx == 0.0


def test_jerk_rms_matches_analytic_value():
    # Known alternating-accel profile -> closed-form jerk.
    # accels = [0, 1, 0, 1, 0, 1, 0] rad/s^2 on joint 0, dt=0.1 s.
    # Each transition produces jerk magnitude 1/0.1 = 10 rad/s^3.
    # rms over the 6 transitions is also 10.0 (all equal magnitude).
    dt = 0.1
    accels = [[0.0] * 7, [1.0] + [0.0] * 6, [0.0] * 7, [1.0] + [0.0] * 6,
              [0.0] * 7, [1.0] + [0.0] * 6, [0.0] * 7]
    positions = [[0.0] * 7 for _ in range(len(accels))]
    traj = _fake_trajectory(positions, accels, dt=dt)
    rms, mx = planning_helpers.trajectory_jerk_metrics(traj)
    # Each transition contributes magnitude 10 on one joint, 0 on
    # other six -> rms = sqrt(6 * 10^2 / 42) = sqrt(100/7) ≈ 3.78.
    assert rms == pytest.approx((100.0 / 7.0) ** 0.5, rel=0.01)
    assert mx == pytest.approx(10.0, rel=0.01)


# ---------------------------------------------------------------------------
# _reachability_score edge cases
# ---------------------------------------------------------------------------

def test_reach_score_peaks_at_sweet_spot():
    pose = _pose(planning_helpers.REACH_SWEET_SPOT_M, 0.0, 0.0)
    assert planning_helpers._reachability_score(pose) == pytest.approx(1.0)


def test_reach_score_zero_beyond_falloff():
    far = planning_helpers.REACH_SWEET_SPOT_M + planning_helpers.REACH_FALLOFF_M + 0.1
    pose = _pose(far, 0.0, 0.0)
    assert planning_helpers._reachability_score(pose) == 0.0


def test_reach_score_survives_zero_falloff(monkeypatch):
    # Degenerate tuning: falloff == 0. Must not divide by zero; instead
    # collapse to a binary match at the sweet spot.
    monkeypatch.setattr(planning_helpers, 'REACH_FALLOFF_M', 0.0)
    hit = _pose(planning_helpers.REACH_SWEET_SPOT_M, 0.0, 0.0)
    miss = _pose(planning_helpers.REACH_SWEET_SPOT_M + 0.1, 0.0, 0.0)
    assert planning_helpers._reachability_score(hit) == 1.0
    assert planning_helpers._reachability_score(miss) == 0.0
