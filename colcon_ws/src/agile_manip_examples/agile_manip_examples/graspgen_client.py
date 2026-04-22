"""GraspGen client that calls the real tutorial service and republishes poses."""

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Empty
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from agile_manip_examples.graspgen_utils import (
    GraspMarkerForwarder,
    collect_grasp_candidates,
    grasp_frames_already_published,
    load_grasp_scores,
    make_identity_transform,
    transform_available,
)

class GraspGenClient(Node):
    """Call ``/generate_grasp`` and republish the resulting TF grasps."""

    def __init__(self):
        super().__init__('graspgen_client')

        self.declare_parameter('grasp_service_name', '/generate_grasp')
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

        self.grasp_pub = self.create_publisher(
            PoseArray, '/graspgen/grasp_poses', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.grasp_client = self.create_client(
            Empty, self.get_parameter('grasp_service_name').value)
        self.pending_collection_timer = None
        self.marker_forwarder = GraspMarkerForwarder(self)

        self.startup_timer = self.create_timer(1.0, self.request_grasps)

    def request_grasps(self):
        """Call the GraspGen service once when it becomes available."""
        if not rclpy.ok():
            return

        # Skip the service call if grasp TFs already exist on /tf_static
        # (start_backends.sh pre-runs /generate_grasp so every demo
        # reuses the same grasps instead of regenerating them).
        if grasp_frames_already_published(
                self.tf_buffer,
                self.get_parameter('object_frame').value,
                self.get_parameter('grasp_frame_prefix').value,
                timeout_sec=0.0):
            self.startup_timer.cancel()
            self.publish_object_identity_tf_if_requested()
            self.get_logger().info(
                'Reusing previously generated GraspGen grasps.')
            self.collect_and_publish_grasps()
            return

        if not self.grasp_client.service_is_ready():
            self.get_logger().info('Waiting for GraspGen service...')
            return

        self.startup_timer.cancel()
        self.publish_object_identity_tf_if_requested()

        self.get_logger().info('Calling GraspGen service...')
        future = self.grasp_client.call_async(Empty.Request())
        future.add_done_callback(self.handle_grasp_service_response)

    def publish_object_identity_tf_if_requested(self):
        """Publish ``world -> object`` identity if no object pose is provided."""
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
        """Schedule TF collection after the GraspGen service finishes."""
        try:
            future.result()
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().error(f'GraspGen service call failed: {error}')
            return

        delay_sec = self.get_parameter('result_collection_delay_sec').value
        self.get_logger().info(
            f'GraspGen service completed. Collecting TF outputs in {delay_sec:.2f} s.')
        self.pending_collection_timer = self.create_timer(
            delay_sec, self.collect_and_publish_grasps)

    def collect_and_publish_grasps(self):
        """Collect grasp TF frames and publish them as a PoseArray."""
        if self.pending_collection_timer is not None:
            self.pending_collection_timer.cancel()
            self.pending_collection_timer = None

        world_frame = self.get_parameter('world_frame').value
        object_frame = self.get_parameter('object_frame').value
        # world -> object is published moments ago by a StaticTransform
        # Broadcaster on this same node; re-try from a timer until the
        # /tf_static subscription has delivered it to our tf_buffer.
        if not transform_available(self.tf_buffer, world_frame, object_frame):
            self._tf_retries = getattr(self, '_tf_retries', 0) + 1
            if self._tf_retries > 30:
                self.get_logger().error(
                    f'TF "{world_frame}" -> "{object_frame}" not available '
                    'after 6s; cannot collect grasps in the world frame.')
                return
            self.pending_collection_timer = self.create_timer(
                0.2, self.collect_and_publish_grasps)
            return
        self._tf_retries = 0
        # Re-emit the forwarded MarkerArray now that the object frame is
        # resolvable: the initial publish (triggered by the latched
        # /grasp_markers delivery) may have reached RViz before the static
        # world->object TF did, which puts the display into an error state.
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
            self.get_logger().error('No grasp TF frames were published by GraspGen.')
            return

        header = Header(
            frame_id=self.get_parameter('world_frame').value,
            stamp=self.get_clock().now().to_msg())
        self.grasp_pub.publish(PoseArray(header=header, poses=grasp_poses))
        best_candidate = max(
            grasp_candidates,
            key=lambda candidate: candidate.confidence
            if candidate.confidence is not None else float('-inf'),
            default=None,
        )
        if best_candidate and best_candidate.confidence is not None:
            self.get_logger().info(
                f'Published {len(grasp_poses)} grasp poses. '
                f'Best grasp: {best_candidate.grasp_id} '
                f'(confidence={best_candidate.confidence:.4f}).')
        else:
            self.get_logger().info(f'Published {len(grasp_poses)} grasp poses.')


def main(args=None):
    rclpy.init(args=args)
    node = GraspGenClient()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
