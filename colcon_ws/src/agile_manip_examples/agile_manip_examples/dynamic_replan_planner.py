"""Dynamic replanning pipeline.

Demonstrates cuMotion's ability to produce a fresh collision-free plan
at a high rate while a dynamic obstacle moves through the scene. The
node:

1. Calls GraspGen once and picks a grasp (``highest_confidence`` or
   ``multi_criteria``, same as the other planners).
2. Registers a repeating timer that, at ``replan_rate_hz``, (a) polls
   the latest obstacle pose from either an externally published topic
   or an internal sinusoidal oscillator, (b) issues a new cuMotion
   plan request with that obstacle attached to the scene diff, and
   (c) republishes the resulting trajectory line marker to RViz.

The robot visualisation is held at the home pose; only the *plan*
updates. This makes the replanning cadence visible in RViz: watch the
blue trajectory line morph as the obstacle oscillates. No trajectory
playback is performed -- it is trivial to add on top and would only
obscure the replan-rate signal we want to showcase.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from agile_manip_examples.cumotion_utils import (
    HOME_ARM_JOINTS,
    HOME_JOINTS,
    JOINT_NAMES,
    build_pose_goal,
    expand_robot_trajectory,
    moveit_error_name,
)
from agile_manip_examples.graspgen_utils import (
    GraspMarkerForwarder,
    collect_grasp_candidates,
    grasp_frames_already_published,
    load_grasp_scores,
    make_identity_transform,
    transform_available,
)
from agile_manip_examples.planning_helpers import (
    build_trajectory_path_marker,
    order_grasp_candidates,
)


class DynamicReplanPlanner(Node):
    """Continuously replan around a moving box obstacle."""

    TOPIC_NAMESPACE = '/dynamic_replan'
    # Default topic on which an external driver can publish the obstacle
    # centre pose. A publisher is not required -- the internal
    # oscillator below is used whenever the cache stays empty.
    DEFAULT_OBSTACLE_POSE_TOPIC = '/dynamic_obstacle/pose'

    def __init__(self):
        super().__init__('dynamic_replan_planner')

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
        self.declare_parameter('allowed_planning_time', 1.0)
        self.declare_parameter('position_tolerance_m', 0.005)
        self.declare_parameter('orientation_tolerance_rad', 0.05)
        self.declare_parameter('selection_mode', 'highest_confidence')
        self.declare_parameter('selected_grasp_index', 0)
        self.declare_parameter('multi_criteria_weight_confidence', 0.7)
        self.declare_parameter('multi_criteria_weight_reach', 0.3)

        # Obstacle geometry -- size is fixed; the centre is updated on
        # every replan tick from either the subscription or the internal
        # oscillator.
        self.declare_parameter('obstacle_size_xyz', [0.08, 0.40, 0.50])
        self.declare_parameter('obstacle_frame', 'world')
        self.declare_parameter('obstacle_pose_topic',
                               self.DEFAULT_OBSTACLE_POSE_TOPIC)

        # Internal oscillator: used when no external pose has been
        # received yet. y(t) = amplitude * sin(2 * pi * frequency * t)
        # around ``oscillator_center_xyz``.
        self.declare_parameter('oscillator_center_xyz', [0.25, 0.0, 0.40])
        self.declare_parameter('oscillator_axis', 'y')   # x | y | z
        self.declare_parameter('oscillator_amplitude_m', 0.22)
        self.declare_parameter('oscillator_frequency_hz', 0.3)

        self.declare_parameter('replan_rate_hz', 5.0)
        self.declare_parameter('publish_obstacle_marker', True)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, f'{self.TOPIC_NAMESPACE}/markers', 10)
        self.obstacle_marker_pub = self.create_publisher(
            Marker, f'{self.TOPIC_NAMESPACE}/obstacle_marker', latched_qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.grasp_client = self.create_client(
            Empty, self.get_parameter('grasp_service_name').value)
        self.move_group_client = ActionClient(
            self, MoveGroup,
            self.get_parameter('move_group_action_name').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        # Forwarder recolours the candidate gripper markers so the
        # selected grasp is visibly highlighted in yellow; without it
        # RViz would just show whatever latched MarkerArray an
        # earlier planner left on /graspgen/object_and_grippers,
        # making the highlight disagree with the actual cuMotion goal.
        self.marker_forwarder = GraspMarkerForwarder(self)

        self._external_obstacle_xyz = None
        self._external_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('obstacle_pose_topic').value,
            self._on_external_obstacle,
            10,
        )

        self._selected_candidate = None
        self._selected_index = None
        self._replan_in_flight = False
        self._replan_count = 0
        self._success_count = 0
        self._last_log_time = time.monotonic()
        self._startup_t0 = time.monotonic()

        self.publish_joint_state(HOME_JOINTS)
        # Continuously re-emit HOME joints so robot_state_publisher keeps
        # a fresh set of TFs alive and RViz's RobotModel display does
        # not go stale while we replan for many seconds. 30 Hz is cheap
        # and matches the typical RViz refresh rate.
        self.create_timer(1.0 / 30.0, lambda: self.publish_joint_state(HOME_JOINTS))
        self.startup_timer = self.create_timer(1.0, self._startup)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def publish_joint_state(self, joint_positions):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = list(joint_positions)
        self.joint_pub.publish(joint_state)

    def _on_external_obstacle(self, msg):
        # Only the xyz is consumed; orientation is always identity.
        self._external_obstacle_xyz = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _startup(self):
        if not rclpy.ok():
            return
        if not self.grasp_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Waiting for GraspGen service...')
            return
        if not self.move_group_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for cuMotion MoveGroup action server...')
            return

        self.startup_timer.cancel()
        self._publish_object_identity_tf_if_requested()

        if not self._prepare_grasp():
            self.get_logger().error(
                'Failed to prepare a grasp candidate; aborting dynamic replan.')
            return

        rate_hz = float(self.get_parameter('replan_rate_hz').value)
        period = 1.0 / max(rate_hz, 0.1)
        self.get_logger().info(
            f'=== Dynamic replanning at {rate_hz:.1f} Hz ===')
        self._startup_t0 = time.monotonic()
        self.create_timer(period, self._replan_tick)

    def _publish_object_identity_tf_if_requested(self):
        if not self.get_parameter('publish_object_identity_tf').value:
            return
        transform = make_identity_transform(
            self.get_parameter('world_frame').value,
            self.get_parameter('object_frame').value,
            self.get_clock().now().to_msg(),
            translation=self.get_parameter('object_pose_xyz').value,
        )
        self.tf_broadcaster.sendTransform(transform)

    def _prepare_grasp(self):
        """Populate ``self._selected_candidate`` by calling GraspGen once."""
        object_frame = self.get_parameter('object_frame').value
        prefix = self.get_parameter('grasp_frame_prefix').value
        world_frame = self.get_parameter('world_frame').value

        if not grasp_frames_already_published(self.tf_buffer, object_frame, prefix):
            self.get_logger().info('Calling GraspGen once to populate grasp TFs.')
            future = self.grasp_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            delay = float(self.get_parameter('result_collection_delay_sec').value)
            end = time.monotonic() + delay
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
        else:
            self.get_logger().info('Reusing previously generated GraspGen grasps.')

        for _ in range(30):
            if transform_available(self.tf_buffer, world_frame, object_frame):
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        score_map = load_grasp_scores(
            self.get_parameter('grasp_result_path').value)
        candidates = collect_grasp_candidates(
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
        if not candidates:
            return False

        ordered = order_grasp_candidates(
            candidates,
            self.get_parameter('selection_mode').value,
            self.get_parameter('selected_grasp_index').value,
            multi_criteria_weights={
                'confidence': self.get_parameter(
                    'multi_criteria_weight_confidence').value,
                'reach': self.get_parameter(
                    'multi_criteria_weight_reach').value,
            },
        )
        self._selected_index, self._selected_candidate = ordered[0]
        # Recolour the chosen gripper marker so the yellow highlight in
        # RViz matches the pose cuMotion is actually being asked to
        # reach. Without this the forwarder's latched MarkerArray from a
        # previous demo could display a different gripper in yellow.
        self.marker_forwarder.highlight_gripper(self._selected_index)
        self.get_logger().info(
            f'Selected grasp index={self._selected_index} '
            f'confidence={self._selected_candidate.confidence}')
        return True

    # ------------------------------------------------------------------
    # Replan loop
    # ------------------------------------------------------------------
    def _current_obstacle_center(self):
        """Return (x, y, z) of the obstacle this tick.

        External pose (subscribed) wins; otherwise the sinusoidal
        oscillator drives motion so the demo is self-contained.
        """
        if self._external_obstacle_xyz is not None:
            return self._external_obstacle_xyz

        center = list(self.get_parameter('oscillator_center_xyz').value)
        axis = self.get_parameter('oscillator_axis').value
        amp = float(self.get_parameter('oscillator_amplitude_m').value)
        freq = float(self.get_parameter('oscillator_frequency_hz').value)
        t = time.monotonic() - self._startup_t0
        offset = amp * math.sin(2.0 * math.pi * freq * t)
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        idx = axis_map.get(str(axis).lower(), 1)
        center[idx] += offset
        return tuple(float(v) for v in center)

    def _build_collision_object(self, center_xyz):
        size = self.get_parameter('obstacle_size_xyz').value
        frame = self.get_parameter('obstacle_frame').value
        stamp = self.get_clock().now().to_msg()

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        pose = Pose()
        pose.position.x = float(center_xyz[0])
        pose.position.y = float(center_xyz[1])
        pose.position.z = float(center_xyz[2])
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = frame
        collision.header.stamp = stamp
        collision.id = 'dynamic_obstacle'
        collision.primitives = [box]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        return collision, pose

    def _publish_obstacle_marker(self, pose):
        if not self.get_parameter('publish_obstacle_marker').value:
            return
        size = self.get_parameter('obstacle_size_xyz').value
        marker = Marker()
        marker.header.frame_id = self.get_parameter('obstacle_frame').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dynamic_obstacle'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])
        marker.color.r = 0.2
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.85
        self.obstacle_marker_pub.publish(marker)

    def _replan_tick(self):
        # Drop the tick if the previous plan is still in flight --
        # stacking goals on top of each other both stresses the action
        # server and makes the replan-rate metric meaningless.
        if self._replan_in_flight or self._selected_candidate is None:
            return

        center = self._current_obstacle_center()
        collision, pose = self._build_collision_object(center)
        self._publish_obstacle_marker(pose)

        goal_msg = build_pose_goal(
            self.get_parameter('planner_group_name').value,
            self.get_parameter('pipeline_id').value,
            self.get_parameter('planner_id').value,
            self.get_parameter('end_effector_link').value,
            self.get_parameter('world_frame').value,
            HOME_ARM_JOINTS,
            self._selected_candidate.pose,
            self.get_parameter('allowed_planning_time').value,
            position_tolerance_m=self.get_parameter('position_tolerance_m').value,
            orientation_tolerance_rad=self.get_parameter(
                'orientation_tolerance_rad').value,
        )
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.world.collision_objects = [
            collision,
        ]

        self._replan_in_flight = True
        self._replan_count += 1
        self._tick_sent_at = time.monotonic()
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().warn(f'send_goal failed: {error}')
            self._replan_in_flight = False
            return
        if handle is None or not handle.accepted:
            self._replan_in_flight = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            outcome = future.result()
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().warn(f'get_result failed: {error}')
            self._replan_in_flight = False
            return

        result = outcome.result if outcome is not None else None
        self._replan_in_flight = False
        elapsed_ms = (time.monotonic() - self._tick_sent_at) * 1000.0

        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            err = (moveit_error_name(result.error_code.val)
                   if result is not None else 'NO_RESULT')
            # Replan failures are expected when the obstacle covers the
            # grasp -- log at a throttled rate so we don't flood the
            # console.
            now = time.monotonic()
            if now - self._last_log_time > 1.0:
                self.get_logger().warn(
                    f'Replan failed ({err}) in {elapsed_ms:.1f} ms; '
                    'waiting for obstacle to move.')
                self._last_log_time = now
            return

        self._success_count += 1
        trajectory = expand_robot_trajectory(result.planned_trajectory)
        self._publish_trajectory_marker(trajectory)

        now = time.monotonic()
        if now - self._last_log_time > 1.0:
            rate = self._replan_count / max(now - self._startup_t0, 1e-3)
            self.get_logger().info(
                f'Replan #{self._replan_count} OK in {elapsed_ms:.1f} ms '
                f'(avg rate {rate:.1f} Hz, '
                f'{self._success_count}/{self._replan_count} ok)')
            self._last_log_time = now

    def _publish_trajectory_marker(self, trajectory):
        marker_array = MarkerArray()
        marker_array.markers.append(build_trajectory_path_marker(
            trajectory,
            self.get_parameter('world_frame').value,
            self.get_clock().now().to_msg(),
        ))
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicReplanPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
