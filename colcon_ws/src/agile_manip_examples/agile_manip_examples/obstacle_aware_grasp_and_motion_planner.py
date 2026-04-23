"""Obstacle-aware GraspGen + cuMotion pipeline.

This example is identical to :mod:`grasp_and_motion_planner` except
that it also injects a static box obstacle into MoveIt's planning scene
before requesting a plan. cuMotion picks the obstacle up through the
planning-scene monitor and routes the arm around it, so the trajectory
visibly differs from the obstacle-free version in ``grasp_and_motion``.

The obstacle is also published as an RViz Marker so the viewer can see
what is being avoided.
"""

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
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
    log_goal_residual,
    order_grasp_candidates,
    resolve_gripper_value,
)


class ObstacleAwareGraspAndMotionPlanner(Node):
    """Run GraspGen + cuMotion with a static box obstacle in the scene."""

    TOPIC_NAMESPACE = '/obstacle_aware_grasp_and_motion'

    def __init__(self):
        super().__init__('obstacle_aware_grasp_and_motion_planner')

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
        self.declare_parameter('selected_grasp_index', 0)
        self.declare_parameter('selection_mode', 'highest_confidence')
        self.declare_parameter('playback_period_sec', 0.05)
        # See grasp_and_motion_planner.py for rationale; defaults match
        # the previous hard-coded values.
        self.declare_parameter('position_tolerance_m', 0.005)
        self.declare_parameter('orientation_tolerance_rad', 0.05)
        # Static box obstacle fed to MoveIt's planning scene. Position
        # and size are tuned so that a naive straight-line approach from
        # the iiwa home pose to the grasp point is blocked, forcing
        # cuMotion to compute a detour that is visibly different from
        # the obstacle-free :mod:`grasp_and_motion_planner`.
        self.declare_parameter('obstacle_center_xyz', [0.25, 0.0, 0.40])
        self.declare_parameter('obstacle_size_xyz', [0.08, 0.40, 0.50])
        self.declare_parameter('obstacle_frame', 'world')
        # Gripper is driven independently of cuMotion. Defaults to open.
        self.declare_parameter('gripper_finger_joint_target', 0.0)
        self.declare_parameter('gripper_width_m', -1.0)

        self.grasp_candidates_pub = self.create_publisher(
            PoseArray, f'{self.TOPIC_NAMESPACE}/grasp_candidates', 10)
        self.selected_pub = self.create_publisher(
            PoseStamped, f'{self.TOPIC_NAMESPACE}/selected_grasp', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, f'{self.TOPIC_NAMESPACE}/markers', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # RViz marker visualising the obstacle (no planner integration).
        self.obstacle_marker_pub = self.create_publisher(
            Marker, f'{self.TOPIC_NAMESPACE}/obstacle_marker', latched_qos)
        # Holds the CollisionObject that we attach to every cuMotion goal
        # via ``goal.planning_options.planning_scene_diff``; the cuMotion
        # action server reads its world-collision updates from there,
        # not from the ``/planning_scene`` topic.
        self._obstacle_collision_object = None

        self.grasp_client = self.create_client(
            Empty, self.get_parameter('grasp_service_name').value)
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            self.get_parameter('move_group_action_name').value,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.marker_forwarder = GraspMarkerForwarder(self)

        self.trajectory = []
        self.current_waypoint_index = 0
        self.playback_timer = None
        self.pending_collection_timer = None
        self.selected_pose = None
        self.pipeline_started = False
        self.remaining_candidates = []
        self.last_header = None

        self.publish_joint_state(HOME_JOINTS)
        self.startup_timer = self.create_timer(1.0, self.start_pipeline)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def publish_joint_state(self, joint_positions):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = list(joint_positions)
        self.joint_pub.publish(joint_state)

    def _gripper_value(self):
        """Resolve the current gripper target (rad) from parameters."""
        return resolve_gripper_value(
            self.get_parameter('gripper_width_m').value,
            self.get_parameter('gripper_finger_joint_target').value,
        )

    # ------------------------------------------------------------------
    # Pipeline
    # ------------------------------------------------------------------
    def start_pipeline(self):
        if self.pipeline_started:
            return
        if not rclpy.ok():
            return
        if not self.grasp_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Waiting for GraspGen service...')
            return
        if not self.move_group_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for cuMotion MoveGroup action server...')
            return

        self.pipeline_started = True
        self.startup_timer.cancel()
        self.publish_object_identity_tf_if_requested()
        self.publish_static_obstacle()
        self.get_logger().info(
            '=== Starting obstacle-aware GraspGen + cuMotion pipeline ===')

        if grasp_frames_already_published(
                self.tf_buffer,
                self.get_parameter('object_frame').value,
                self.get_parameter('grasp_frame_prefix').value):
            self.get_logger().info(
                'Reusing previously generated GraspGen grasps.')
            self.collect_grasps_and_plan()
            return

        future = self.grasp_client.call_async(Empty.Request())
        future.add_done_callback(self.handle_grasp_service_response)

    def publish_object_identity_tf_if_requested(self):
        if not self.get_parameter('publish_object_identity_tf').value:
            return
        transform = make_identity_transform(
            self.get_parameter('world_frame').value,
            self.get_parameter('object_frame').value,
            self.get_clock().now().to_msg(),
            translation=self.get_parameter('object_pose_xyz').value,
        )
        self.tf_broadcaster.sendTransform(transform)

    def publish_static_obstacle(self):
        """Build the obstacle CollisionObject and visualise it in RViz.

        The CollisionObject is attached to each cuMotion goal request
        (``planning_options.planning_scene_diff``) in
        :meth:`plan_next_candidate`; cuMotion's action server consumes
        world-collision updates from that field rather than from the
        ``/planning_scene`` topic, so attaching it per-goal is what
        actually makes cuMotion route around the box.
        """
        center = self.get_parameter('obstacle_center_xyz').value
        size = self.get_parameter('obstacle_size_xyz').value
        frame = self.get_parameter('obstacle_frame').value
        stamp = self.get_clock().now().to_msg()

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = frame
        collision.header.stamp = stamp
        collision.id = 'demo_static_obstacle'
        collision.primitives = [box]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        self._obstacle_collision_object = collision

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = 'static_obstacle'
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
        self.get_logger().info(
            f'Configured obstacle box at ({center[0]:.2f},{center[1]:.2f},'
            f'{center[2]:.2f}) size=({size[0]:.2f},{size[1]:.2f},{size[2]:.2f}); '
            'it will be attached to each cuMotion goal.')

    def handle_grasp_service_response(self, future):
        try:
            future.result()
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().error(f'GraspGen service call failed: {error}')
            return
        delay_sec = self.get_parameter('result_collection_delay_sec').value
        self.pending_collection_timer = self.create_timer(
            delay_sec, self.collect_grasps_and_plan)

    def collect_grasps_and_plan(self):
        if self.pending_collection_timer is not None:
            self.pending_collection_timer.cancel()
            self.pending_collection_timer = None

        world_frame = self.get_parameter('world_frame').value
        object_frame = self.get_parameter('object_frame').value
        if not transform_available(self.tf_buffer, world_frame, object_frame):
            self._tf_retries = getattr(self, '_tf_retries', 0) + 1
            if self._tf_retries > 30:
                self.get_logger().error(
                    f'TF "{world_frame}" -> "{object_frame}" not available '
                    'after 6s; cannot plan in the world frame.')
                return
            self.pending_collection_timer = self.create_timer(
                0.2, self.collect_grasps_and_plan)
            return
        self._tf_retries = 0
        self.marker_forwarder.refresh()

        score_map = load_grasp_scores(
            self.get_parameter('grasp_result_path').value)
        grasp_candidates = collect_grasp_candidates(
            self,
            self.tf_buffer,
            target_frame=self.get_parameter('world_frame').value,
            object_frame=self.get_parameter('object_frame').value,
            grasp_frame_prefix=self.get_parameter('grasp_frame_prefix').value,
            max_grasps=self.get_parameter('max_grasps').value,
            max_consecutive_misses=self.get_parameter('max_consecutive_misses').value,
            tf_lookup_timeout_sec=self.get_parameter('tf_lookup_timeout_sec').value,
            score_map=score_map,
        )
        grasp_poses = [candidate.pose for candidate in grasp_candidates]
        if not grasp_poses:
            self.get_logger().error(
                'GraspGen did not publish any reachable grasp TFs.')
            return

        header = Header(
            frame_id=self.get_parameter('world_frame').value,
            stamp=self.get_clock().now().to_msg())
        self.grasp_candidates_pub.publish(
            PoseArray(header=header, poses=grasp_poses))
        self.publish_candidate_markers(header, grasp_poses)
        self.last_header = header

        self.remaining_candidates = self.ordered_grasp_candidates(grasp_candidates)
        self.plan_next_candidate()

    def ordered_grasp_candidates(self, grasp_candidates):
        return order_grasp_candidates(
            grasp_candidates,
            self.get_parameter('selection_mode').value,
            self.get_parameter('selected_grasp_index').value,
        )

    def plan_next_candidate(self):
        if not self.remaining_candidates:
            self.get_logger().error(
                'Exhausted all GraspGen candidates; no reachable grasp found.')
            return

        selected_index, selected_candidate = self.remaining_candidates.pop(0)
        self.selected_pose = selected_candidate.pose
        header = self.last_header
        self.selected_pub.publish(
            PoseStamped(header=header, pose=self.selected_pose))
        self.marker_forwarder.highlight_gripper(selected_index)

        goal_msg = build_pose_goal(
            self.get_parameter('planner_group_name').value,
            self.get_parameter('pipeline_id').value,
            self.get_parameter('planner_id').value,
            self.get_parameter('end_effector_link').value,
            self.get_parameter('world_frame').value,
            HOME_ARM_JOINTS,
            self.selected_pose,
            self.get_parameter('allowed_planning_time').value,
            position_tolerance_m=self.get_parameter('position_tolerance_m').value,
            orientation_tolerance_rad=self.get_parameter('orientation_tolerance_rad').value,
        )
        if self._obstacle_collision_object is not None:
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.world.collision_objects = [
                self._obstacle_collision_object,
            ]
        self.get_logger().info(
            f'Sending grasp {selected_index} to cuMotion '
            f'(confidence={selected_candidate.confidence}, '
            f'remaining_fallbacks={len(self.remaining_candidates)}).')
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_move_group_goal_response)

    def handle_move_group_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().error(f'Failed to send cuMotion goal: {error}')
            self.plan_next_candidate()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('cuMotion rejected the grasp pose goal; trying next candidate.')
            self.plan_next_candidate()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_plan_result)

    def handle_plan_result(self, future):
        try:
            result = future.result().result
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().error(f'Failed to receive cuMotion result: {error}')
            self.plan_next_candidate()
            return
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f'cuMotion rejected candidate with {moveit_error_name(error_code)}; '
                'trying next candidate.')
            self.plan_next_candidate()
            return

        self.trajectory = expand_robot_trajectory(
            result.planned_trajectory,
            gripper_value=self._gripper_value())
        if not self.trajectory:
            self.get_logger().error('cuMotion returned an empty trajectory.')
            return
        self.publish_trajectory_markers()
        self.get_logger().info(
            f'Obstacle-aware pipeline succeeded with {len(self.trajectory)} '
            'waypoints.')
        self.playback_timer = self.create_timer(
            self.get_parameter('playback_period_sec').value,
            self.publish_next_waypoint,
        )

    # ------------------------------------------------------------------
    # Visualization helpers
    # ------------------------------------------------------------------
    def publish_candidate_markers(self, header, grasp_poses):
        marker_array = MarkerArray()
        for grasp_index, grasp_pose in enumerate(grasp_poses):
            marker = Marker()
            marker.header = header
            marker.ns = 'grasp_candidates'
            marker.id = grasp_index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = grasp_pose
            marker.scale.x = 0.015
            marker.scale.y = 0.015
            marker.scale.z = 0.015
            marker.color.r = 0.0
            marker.color.g = 0.9
            marker.color.b = 0.3
            marker.color.a = 0.6
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_trajectory_markers(self):
        marker_array = MarkerArray()
        marker_array.markers.append(build_trajectory_path_marker(
            self.trajectory,
            self.get_parameter('world_frame').value,
            self.get_clock().now().to_msg(),
        ))
        self.marker_pub.publish(marker_array)

    def publish_next_waypoint(self):
        if not self.trajectory:
            return
        if self.current_waypoint_index >= len(self.trajectory):
            if self.playback_timer is not None:
                self.playback_timer.cancel()
                self.playback_timer = None
            self._verify_goal_reached()
            return
        self.publish_joint_state(self.trajectory[self.current_waypoint_index])
        self.current_waypoint_index += 1

    def _verify_goal_reached(self):
        log_goal_residual(
            self.get_logger(), self.trajectory, self.selected_pose)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAwareGraspAndMotionPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
