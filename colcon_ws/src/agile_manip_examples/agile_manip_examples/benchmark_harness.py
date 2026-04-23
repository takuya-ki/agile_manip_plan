"""Benchmark harness for the GraspGen + cuMotion pipeline.

Runs the same GraspGen -> cuMotion plan request ``iterations`` times
and writes per-iteration metrics to a CSV. Designed so the user can
compare selection modes, tolerances, or future planner options on a
common workload and quote numerical results (planning time, success
rate, trajectory smoothness) for the ``ultra-fast planning`` claim.

This node deliberately does **not** subclass either existing planner
so that the demos remain untouched. It implements a minimal client
that mirrors their control flow.
"""

import csv
import math
import os
import statistics
import time
from datetime import datetime

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from agile_manip_examples.cumotion_utils import (
    HOME_ARM_JOINTS,
    build_pose_goal,
    expand_robot_trajectory,
    grasp_frame_xyz,
    moveit_error_name,
)
from agile_manip_examples.graspgen_utils import (
    collect_grasp_candidates,
    grasp_frames_already_published,
    load_grasp_scores,
    make_identity_transform,
    transform_available,
)
from agile_manip_examples.planning_helpers import (
    goal_residual_m,
    order_grasp_candidates,
)


CSV_COLUMNS = (
    'iteration',
    'selection_mode',
    'selected_grasp_index',
    'grasp_confidence',
    'planning_time_ms',
    'num_waypoints',
    'trajectory_length_m',
    'final_residual_mm',
    'error_code',
    'success',
)


def _trajectory_length_m(trajectory):
    """Cumulative Euclidean path length traced by the grasp_frame in metres.

    ``trajectory`` is the ordered [joints_0..joints_N] list produced by
    :func:`expand_robot_trajectory`, so joint order is guaranteed to
    match ``ARM_JOINT_NAMES`` and the FK call is safe.
    """
    if len(trajectory) < 2:
        return 0.0
    prev = grasp_frame_xyz(trajectory[0])
    total = 0.0
    for joints in trajectory[1:]:
        cur = grasp_frame_xyz(joints)
        total += math.sqrt(sum((a - b) ** 2 for a, b in zip(cur, prev)))
        prev = cur
    return total


class BenchmarkHarness(Node):
    """Iterate the GraspGen + cuMotion pipeline and log per-run metrics."""

    def __init__(self):
        super().__init__('benchmark_harness')

        # Pipeline connection parameters -- match the existing planner
        # nodes so the same launch / backend setup can be reused.
        self.declare_parameter('grasp_service_name', '/generate_grasp')
        self.declare_parameter('move_group_action_name', 'cumotion/move_group')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('object_frame', 'object')
        self.declare_parameter('grasp_frame_prefix', 'grasp_')
        self.declare_parameter('max_grasps', 32)
        self.declare_parameter('max_consecutive_misses', 3)
        self.declare_parameter('tf_lookup_timeout_sec', 0.2)
        self.declare_parameter('result_collection_delay_sec', 0.5)
        self.declare_parameter('grasp_result_path', '')
        self.declare_parameter('publish_object_identity_tf', True)
        self.declare_parameter('object_pose_xyz', [0.5, 0.0, 0.15])
        self.declare_parameter('planner_group_name', 'arm')
        self.declare_parameter('pipeline_id', 'isaac_ros_cumotion')
        self.declare_parameter('planner_id', 'cuMotion')
        self.declare_parameter('end_effector_link', 'grasp_frame')
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('position_tolerance_m', 0.005)
        self.declare_parameter('orientation_tolerance_rad', 0.05)

        # Selection mode + multi_criteria weights (see planning_helpers).
        self.declare_parameter('selection_mode', 'highest_confidence')
        self.declare_parameter('selected_grasp_index', 0)
        self.declare_parameter('multi_criteria_weight_confidence', 0.7)
        self.declare_parameter('multi_criteria_weight_reach', 0.3)

        # Benchmark control.
        self.declare_parameter('iterations', 20)
        self.declare_parameter(
            'output_csv',
            f"/tmp/benchmark_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.declare_parameter('sleep_between_iterations_sec', 0.2)

        self.grasp_client = self.create_client(
            Empty, self.get_parameter('grasp_service_name').value)
        self.move_group_client = ActionClient(
            self, MoveGroup,
            self.get_parameter('move_group_action_name').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self._cached_candidates = None
        self._results = []

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------
    def wait_for_backends(self, timeout_sec=30.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if (self.grasp_client.wait_for_service(timeout_sec=0.1)
                    and self.move_group_client.wait_for_server(timeout_sec=0.1)):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def publish_object_tf_if_requested(self):
        if not self.get_parameter('publish_object_identity_tf').value:
            return
        transform = make_identity_transform(
            self.get_parameter('world_frame').value,
            self.get_parameter('object_frame').value,
            self.get_clock().now().to_msg(),
            translation=self.get_parameter('object_pose_xyz').value,
        )
        self.tf_broadcaster.sendTransform(transform)

    def ensure_grasps_cached(self):
        """Make sure /tf_static contains grasp frames; call GraspGen once if not."""
        object_frame = self.get_parameter('object_frame').value
        prefix = self.get_parameter('grasp_frame_prefix').value
        world_frame = self.get_parameter('world_frame').value

        # Spin briefly so the StaticTransformBroadcaster publication lands
        # in our own TF buffer before lookups.
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if transform_available(self.tf_buffer, world_frame, object_frame):
                break

        if grasp_frames_already_published(self.tf_buffer, object_frame, prefix):
            self.get_logger().info('Reusing previously generated GraspGen grasps.')
        else:
            self.get_logger().info('Calling GraspGen once to populate grasp TFs.')
            future = self.grasp_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            # Give StaticTransformBroadcaster a moment to flush.
            delay = float(self.get_parameter('result_collection_delay_sec').value)
            end = time.monotonic() + delay
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

        for _ in range(30):
            if transform_available(self.tf_buffer, world_frame, object_frame):
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        score_map = load_grasp_scores(
            self.get_parameter('grasp_result_path').value)
        self._cached_candidates = collect_grasp_candidates(
            self,
            self.tf_buffer,
            target_frame=world_frame,
            object_frame=object_frame,
            grasp_frame_prefix=prefix,
            max_grasps=self.get_parameter('max_grasps').value,
            max_consecutive_misses=self.get_parameter('max_consecutive_misses').value,
            tf_lookup_timeout_sec=self.get_parameter('tf_lookup_timeout_sec').value,
            score_map=score_map,
        )
        if not self._cached_candidates:
            raise RuntimeError('No grasp candidates available; aborting benchmark.')

    # ------------------------------------------------------------------
    # Benchmark loop
    # ------------------------------------------------------------------
    def _send_goal_blocking(self, goal_msg):
        send_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        outcome = result_future.result()
        return outcome.result if outcome is not None else None

    def run_single_iteration(self, iteration_index):
        """Plan once and return a metrics dict (even on failure)."""
        selection_mode = self.get_parameter('selection_mode').value
        ordered = order_grasp_candidates(
            self._cached_candidates,
            selection_mode,
            self.get_parameter('selected_grasp_index').value,
            multi_criteria_weights={
                'confidence': self.get_parameter(
                    'multi_criteria_weight_confidence').value,
                'reach': self.get_parameter(
                    'multi_criteria_weight_reach').value,
            },
        )

        # Try candidates in order; stop at the first successful plan.
        # Planning time is summed across attempts so the metric reflects
        # "total time to produce a valid trajectory", which is what an
        # end-user feels.
        cumulative_ms = 0.0
        last_error = MoveItErrorCodes.FAILURE
        for selected_idx, candidate in ordered:
            goal_msg = build_pose_goal(
                self.get_parameter('planner_group_name').value,
                self.get_parameter('pipeline_id').value,
                self.get_parameter('planner_id').value,
                self.get_parameter('end_effector_link').value,
                self.get_parameter('world_frame').value,
                HOME_ARM_JOINTS,
                candidate.pose,
                self.get_parameter('allowed_planning_time').value,
                position_tolerance_m=self.get_parameter('position_tolerance_m').value,
                orientation_tolerance_rad=self.get_parameter(
                    'orientation_tolerance_rad').value,
            )
            start = time.monotonic()
            result = self._send_goal_blocking(goal_msg)
            cumulative_ms += (time.monotonic() - start) * 1000.0

            if result is None:
                last_error = MoveItErrorCodes.FAILURE
                continue
            last_error = result.error_code.val
            if last_error != MoveItErrorCodes.SUCCESS:
                continue

            # ``expand_robot_trajectory`` re-orders each waypoint into
            # ``ARM_JOINT_NAMES`` order (plus a gripper column), which
            # matches what :func:`grasp_frame_xyz` expects; reading
            # ``point.positions`` directly would be wrong if cuMotion
            # returned the joints in a different order.
            trajectory = expand_robot_trajectory(result.planned_trajectory)
            num_waypoints = len(trajectory)
            trajectory_len = _trajectory_length_m(trajectory)
            residual_m = goal_residual_m(trajectory[-1], candidate.pose)
            return {
                'iteration': iteration_index,
                'selection_mode': selection_mode,
                'selected_grasp_index': selected_idx,
                'grasp_confidence': (
                    '' if candidate.confidence is None
                    else f'{candidate.confidence:.6f}'),
                'planning_time_ms': f'{cumulative_ms:.2f}',
                'num_waypoints': num_waypoints,
                'trajectory_length_m': f'{trajectory_len:.4f}',
                'final_residual_mm': f'{residual_m * 1000.0:.2f}',
                'error_code': moveit_error_name(last_error),
                'success': '1',
            }

        return {
            'iteration': iteration_index,
            'selection_mode': selection_mode,
            'selected_grasp_index': -1,
            'grasp_confidence': '',
            'planning_time_ms': f'{cumulative_ms:.2f}',
            'num_waypoints': 0,
            'trajectory_length_m': '0.0',
            'final_residual_mm': '',
            'error_code': moveit_error_name(last_error),
            'success': '0',
        }

    def run(self):
        self.publish_object_tf_if_requested()
        if not self.wait_for_backends():
            raise RuntimeError('Backends did not become ready in time.')
        self.ensure_grasps_cached()

        iterations = int(self.get_parameter('iterations').value)
        csv_path = str(self.get_parameter('output_csv').value)
        os.makedirs(os.path.dirname(csv_path) or '.', exist_ok=True)

        self.get_logger().info(
            f'Running {iterations} iterations; output -> {csv_path}')

        with open(csv_path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for i in range(iterations):
                metrics = self.run_single_iteration(i)
                writer.writerow(metrics)
                handle.flush()
                self._results.append(metrics)
                self.get_logger().info(
                    f"[{i + 1}/{iterations}] "
                    f"success={metrics['success']} "
                    f"t={metrics['planning_time_ms']} ms "
                    f"grasp={metrics['selected_grasp_index']} "
                    f"err={metrics['error_code']}")
                time.sleep(float(
                    self.get_parameter('sleep_between_iterations_sec').value))

        self._log_summary(csv_path)

    def _log_summary(self, csv_path):
        successful = [r for r in self._results if r['success'] == '1']
        n = len(self._results)
        log = self.get_logger()
        log.info(f'=== Benchmark summary ({n} iterations) ===')
        log.info(f'  success_rate: {len(successful)}/{n} '
                 f'({100.0 * len(successful) / max(n, 1):.1f}%)')
        if successful:
            times = [float(r['planning_time_ms']) for r in successful]
            lengths = [float(r['trajectory_length_m']) for r in successful]
            log.info(f'  planning_time_ms  median={statistics.median(times):.2f} '
                     f'mean={statistics.mean(times):.2f} '
                     f'min={min(times):.2f} max={max(times):.2f}')
            log.info(f'  trajectory_len_m  median={statistics.median(lengths):.3f} '
                     f'mean={statistics.mean(lengths):.3f}')
        log.info(f'  csv: {csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkHarness()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        node.get_logger().error(f'Benchmark aborted: {error}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
