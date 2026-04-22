"""Integrated GraspGen -> cuMotion example using the real ROS interfaces."""

import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Empty
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from agile_manip_examples.cumotion_utils import (
    GRIPPER_OPEN,
    HOME_ARM_JOINTS,
    HOME_JOINTS,
    JOINT_NAMES,
    build_pose_goal,
    expand_robot_trajectory,
    grasp_frame_xyz,
    moveit_error_name,
    width_to_finger_joint,
)
from agile_manip_examples.graspgen_utils import (
    GraspMarkerForwarder,
    collect_grasp_candidates,
    grasp_frames_already_published,
    load_grasp_scores,
    make_identity_transform,
    transform_available,
)

class GraspAndMotionPlanner(Node):
    """Call GraspGen, select a grasp, then request a cuMotion plan."""

    TOPIC_NAMESPACE = '/grasp_and_motion'

    def __init__(self):
        super().__init__('grasp_and_motion_planner')

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
        # Gripper is driven independently of cuMotion (which only plans
        # the 7 iiwa joints). Keep it fully open during the demo by
        # default; set ``gripper_finger_joint_target`` (rad, 0 .. ~0.7)
        # or ``gripper_width_m`` (0 .. 0.14 m) to close on approach.
        self.declare_parameter('gripper_finger_joint_target', 0.0)
        self.declare_parameter('gripper_width_m', -1.0)

        self.grasp_pub = self.create_publisher(
            PoseArray, f'{self.TOPIC_NAMESPACE}/grasp_candidates', 10)
        self.selected_pub = self.create_publisher(
            PoseStamped, f'{self.TOPIC_NAMESPACE}/selected_grasp', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, f'{self.TOPIC_NAMESPACE}/markers', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

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
        self.remaining_candidates = []
        self.last_header = None

        self.publish_joint_state(HOME_JOINTS)
        self.startup_timer = self.create_timer(1.0, self.start_pipeline)

    def publish_joint_state(self, joint_positions):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = list(joint_positions)
        self.joint_pub.publish(joint_state)

    def _gripper_value(self):
        """Resolve the current gripper target (rad) from parameters."""
        width = float(self.get_parameter('gripper_width_m').value)
        if width >= 0.0:
            return width_to_finger_joint(width)
        return float(self.get_parameter('gripper_finger_joint_target').value)

    def start_pipeline(self):
        """Wait for both backends and kick off GraspGen first."""
        if not rclpy.ok():
            return
        if not self.grasp_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Waiting for GraspGen service...')
            return
        try:
            move_group_ready = self.move_group_client.wait_for_server(timeout_sec=0.1)
        except Exception:
            return
        if not move_group_ready:
            self.get_logger().info('Waiting for cuMotion MoveGroup action server...')
            return

        self.startup_timer.cancel()
        self.publish_object_identity_tf_if_requested()

        # Reuse pre-generated grasps when the service was already called
        # by ``utils/start_backends.sh``; otherwise call /generate_grasp.
        if grasp_frames_already_published(
                self.tf_buffer,
                self.get_parameter('object_frame').value,
                self.get_parameter('grasp_frame_prefix').value):
            self.get_logger().info(
                'Reusing previously generated GraspGen grasps.')
            self.collect_grasps_and_plan()
            return

        self.get_logger().info('Calling GraspGen before requesting cuMotion.')
        future = self.grasp_client.call_async(Empty.Request())
        future.add_done_callback(self.handle_grasp_service_response)

    def publish_object_identity_tf_if_requested(self):
        """Publish ``world -> object`` identity when no object pose exists."""
        if not self.get_parameter('publish_object_identity_tf').value:
            return

        transform = make_identity_transform(
            self.get_parameter('world_frame').value,
            self.get_parameter('object_frame').value,
            self.get_clock().now().to_msg(),
            translation=self.get_parameter('object_pose_xyz').value,
        )
        self.tf_broadcaster.sendTransform(transform)

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
        """Read grasp TFs, choose one, and send it to cuMotion."""
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
        # Re-emit the forwarded MarkerArray now that the object frame is
        # resolvable (see graspgen_client.py for rationale).
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
            self.get_logger().error('GraspGen did not publish any reachable grasp TFs.')
            return

        header = Header(
            frame_id=self.get_parameter('world_frame').value,
            stamp=self.get_clock().now().to_msg())
        self.grasp_pub.publish(PoseArray(header=header, poses=grasp_poses))
        self.last_header = header

        # Queue every candidate so that if cuMotion rejects the top pick
        # (NO_IK_SOLUTION, GOAL_IN_COLLISION, ...) we transparently fall
        # back to the next-best grasp instead of aborting the pipeline.
        self.remaining_candidates = self.ordered_grasp_candidates(grasp_candidates)
        self.plan_next_candidate()

    def ordered_grasp_candidates(self, grasp_candidates):
        """Return ``(original_index, candidate)`` pairs in selection order."""
        selection_mode = self.get_parameter('selection_mode').value
        indexed = list(enumerate(grasp_candidates))
        if selection_mode == 'highest_confidence':
            scored = [(i, c) for i, c in indexed if c.confidence is not None]
            unscored = [(i, c) for i, c in indexed if c.confidence is None]
            scored.sort(key=lambda entry: entry[1].confidence, reverse=True)
            return scored + unscored

        start_index = min(
            max(0, int(self.get_parameter('selected_grasp_index').value)),
            len(grasp_candidates) - 1,
        )
        return indexed[start_index:] + indexed[:start_index]

    def plan_next_candidate(self):
        """Pop the next candidate from the queue and request a plan."""
        if not self.remaining_candidates:
            self.get_logger().error(
                'Exhausted all GraspGen candidates; no reachable grasp found.')
            return

        selected_index, selected_candidate = self.remaining_candidates.pop(0)
        self.selected_pose = selected_candidate.pose
        header = self.last_header
        self.selected_pub.publish(PoseStamped(header=header, pose=self.selected_pose))
        # Highlight the chosen grasp by recolouring its gripper mesh red
        # in the forwarded MarkerArray rather than drawing a separate arrow.
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
        )
        self.get_logger().info(
            f'Sending grasp {selected_index} to cuMotion '
            f'(ee_link={self.get_parameter("end_effector_link").value}, '
            f'confidence={selected_candidate.confidence}, '
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
            f'Integrated pipeline succeeded with {len(self.trajectory)} waypoints.')
        self.playback_timer = self.create_timer(
            self.get_parameter('playback_period_sec').value,
            self.publish_next_waypoint,
        )

    def publish_trajectory_markers(self):
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        path_marker = Marker()
        path_marker.header.frame_id = self.get_parameter('world_frame').value
        path_marker.header.stamp = stamp
        path_marker.ns = 'trajectory_path'
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.005
        path_marker.color.r = 0.0
        path_marker.color.g = 0.6
        path_marker.color.b = 1.0
        path_marker.color.a = 0.8
        for joints in self.trajectory:
            x, y, z = grasp_frame_xyz(joints)
            path_marker.points.append(Point(x=x, y=y, z=z))
        marker_array.markers.append(path_marker)

        # The selected grasp itself is highlighted on the gripper mesh
        # via ``marker_forwarder.highlight_gripper``; no extra marker needed.
        self.marker_pub.publish(marker_array)

    def publish_next_waypoint(self):
        """Play trajectory forward once and hold at the chosen grasp."""
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
        """Log gripper-base-to-grasp residual (the link cuMotion targeted)."""
        if self.selected_pose is None:
            return
        final = self.trajectory[-1]
        bx, by, bz = grasp_frame_xyz(final)
        gx = self.selected_pose.position.x
        gy = self.selected_pose.position.y
        gz = self.selected_pose.position.z
        residual = ((bx - gx) ** 2 + (by - gy) ** 2 + (bz - gz) ** 2) ** 0.5
        if residual < 0.02:
            self.get_logger().info(
                f'Reached selected grasp pose (gripper-base residual '
                f'{residual*1000:.1f} mm).')
        else:
            self.get_logger().warn(
                f'Trajectory end is {residual*1000:.1f} mm away from the '
                f'selected grasp pose (gripper_base=({bx:.3f},{by:.3f},'
                f'{bz:.3f}), grasp=({gx:.3f},{gy:.3f},{gz:.3f})).')


def main(args=None):
    rclpy.init(args=args)
    node = GraspAndMotionPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
