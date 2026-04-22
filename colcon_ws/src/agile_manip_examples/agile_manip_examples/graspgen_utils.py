"""Helpers for consuming GraspGen tutorial outputs."""

from dataclasses import dataclass
from pathlib import Path

from geometry_msgs.msg import Pose, TransformStamped
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from visualization_msgs.msg import MarkerArray
import yaml


class GraspMarkerForwarder:
    """Cache the volatile ``/grasp_markers`` publication and re-publish it latched.

    ``graspgen_ros`` (``mesh_graspgen_service``) publishes its
    ``MarkerArray`` once per service call with default QoS (RELIABLE +
    VOLATILE), which means RViz misses the update unless it happened to
    subscribe before the single publish. Mirroring the marker array to a
    TRANSIENT_LOCAL topic lets late subscribers (RViz started alongside
    the demo launch) still see the object mesh and candidate grippers.

    This forwarder also restyles the markers for clearer visualization:
    the target object is painted a bright colour, candidate grippers are
    drawn translucent green, and the selected grasp's gripper is swapped
    for a fully-opaque highlight colour so it stands out in the crowd.
    """

    DEFAULT_SOURCE_TOPIC = '/grasp_markers'
    DEFAULT_FORWARD_TOPIC = '/graspgen/object_and_grippers'
    OBJECT_NS = 'object'
    OBJECT_FRAME = 'object'
    GRIPPER_NS = 'gripper'

    # Visible defaults tuned against the iiwa14 white / grey meshes.
    OBJECT_RGBA = (0.9, 0.1, 0.1, 1.0)        # bright red object mesh
    CANDIDATE_RGBA = (0.0, 0.85, 0.25, 0.15)  # translucent green gripper mesh
    # Highlighted gripper is translucent (~0.7) so it sits visibly on
    # top of the candidate cluster without fully hiding the red object
    # mesh that shares its pose; opaque yellow was masking the target.
    HIGHLIGHT_RGBA = (1.0, 0.8, 0.0, 0.7)

    def __init__(self, node, source_topic=DEFAULT_SOURCE_TOPIC,
                 forward_topic=DEFAULT_FORWARD_TOPIC):
        self._node = node
        self._latest = None
        self._highlight_index = None
        self._highlight_rgba = self.HIGHLIGHT_RGBA

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = node.create_publisher(MarkerArray, forward_topic, latched_qos)
        # Match the TRANSIENT_LOCAL publisher in graspgen_ros so a client
        # started after pre-warming still receives the most recent grasp
        # MarkerArray from the latched topic.
        self._sub = node.create_subscription(
            MarkerArray, source_topic, self._on_markers, latched_qos)

    def _on_markers(self, msg):
        self._latest = msg
        self._pub.publish(self._styled(msg))

    def _styled(self, msg):
        """Return a re-coloured copy of ``msg``.

        Only colours are changed; the mesh geometry and poses are
        preserved so every candidate grasp renders as the same gripper
        model that graspgen_ros published.

        Rendering order (RViz draws markers in array order, later
        markers blended on top):

        1. Candidate gripper meshes (very translucent green) -- background.
        2. Highlighted gripper (translucent yellow) -- stands out above
           the candidate cluster.
        3. Object mesh (opaque red) -- drawn last so the target stays
           fully visible on top of the overlapping gripper geometry.
        """
        import copy

        styled = MarkerArray()
        object_marker = None
        highlighted = None
        for marker in msg.markers:
            m = copy.deepcopy(marker)
            m.mesh_use_embedded_materials = False
            if m.ns == self.OBJECT_NS:
                m.header.frame_id = self.OBJECT_FRAME
                m.pose.position.x = 0.0
                m.pose.position.y = 0.0
                m.pose.position.z = 0.0
                self._set_rgba(m, self.OBJECT_RGBA)
                object_marker = m
            elif m.ns == self.GRIPPER_NS:
                is_highlight = (
                    self._highlight_index is not None
                    and m.id == self._highlight_index)
                if is_highlight:
                    self._set_rgba(m, self._highlight_rgba)
                    highlighted = m
                else:
                    self._set_rgba(m, self.CANDIDATE_RGBA)
                    styled.markers.append(m)
            else:
                styled.markers.append(m)
        if highlighted is not None:
            styled.markers.append(highlighted)
        if object_marker is not None:
            styled.markers.append(object_marker)
        return styled

    @staticmethod
    def _set_rgba(marker, rgba):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba

    def highlight_gripper(self, index, rgba=None):
        """Flag gripper ``index`` as the selected grasp and republish."""
        self._highlight_index = index
        if rgba is not None:
            self._highlight_rgba = rgba
        if self._latest is not None:
            self._pub.publish(self._styled(self._latest))

    def clear_highlight(self):
        self._highlight_index = None
        if self._latest is not None:
            self._pub.publish(self._styled(self._latest))

    def refresh(self):
        """Re-emit the cached MarkerArray.

        The forwarder publishes immediately when /grasp_markers arrives,
        but RViz may have received that message before the world -> object
        TF was visible, which flags the MarkerArray display with a
        frame-resolution error icon. Calling ``refresh`` once the TF is
        known pushes a fresh copy so the display reconciles and draws
        the object + grippers.
        """
        if self._latest is not None:
            self._pub.publish(self._styled(self._latest))


@dataclass
class GraspCandidate:
    """Grasp pose with optional metadata loaded from GraspGen outputs."""

    frame_id: str
    grasp_id: str
    pose: Pose
    confidence: float | None = None


def _transform_to_pose_components(transform_stamped):
    transform = transform_stamped.transform
    return (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )


def transform_to_pose(transform_stamped):
    """Convert a TF transform into a ``geometry_msgs/Pose``."""
    tx, ty, tz, qx, qy, qz, qw = _transform_to_pose_components(transform_stamped)
    pose = Pose()
    pose.position.x = tx
    pose.position.y = ty
    pose.position.z = tz
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def make_identity_transform(parent_frame, child_frame, stamp, translation=(0.0, 0.0, 0.0)):
    """Create a transform with identity orientation and optional translation.

    The translation defaults to zero (identity). When the object is placed at
    the robot base the grasp poses fall at the robot origin and motion
    planning cannot reach them; callers can pass a reachable ``(x, y, z)``
    to position the object within the arm's workspace.
    """
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation.w = 1.0
    return transform


def transform_available(tf_buffer, target_frame, source_frame):
    """Return True if ``target_frame <- source_frame`` is in the buffer now.

    StaticTransformBroadcaster latches on /tf_static but the local
    tf_buffer only receives that message on the next executor iteration.
    Callers inside a timer/service callback cannot spin, so this helper
    only does a non-blocking check; schedule a retry via ``create_timer``
    when it returns False rather than busy-waiting.
    """
    return tf_buffer.can_transform(target_frame, source_frame, Time())


def grasp_frames_already_published(tf_buffer, object_frame, grasp_frame_prefix,
                                   timeout_sec=1.0):
    """Return True if the first grasp frame is already on /tf_static.

    ``start_backends.sh`` pre-calls ``/generate_grasp`` so the grasp
    static transforms are broadcast up front; each demo client can then
    reuse them instead of triggering another inference pass. This
    helper lets a client skip the service call cheaply when the
    expected ``grasp_0`` frame is already visible.
    """
    first_frame = f'{grasp_frame_prefix}0'
    try:
        tf_buffer.lookup_transform(
            object_frame, first_frame, Time(),
            timeout=Duration(seconds=float(timeout_sec)))
    except TransformException:
        return False
    return True


def load_grasp_scores(result_path):
    """Load confidence scores from a GraspGen Isaac-format YAML file.

    The upstream ``save_to_isaac_grasp_format`` writer occasionally emits
    malformed indentation for negative-float list entries, so we fall back
    to a regex scan when PyYAML rejects the file. Missing or unreadable
    files just return no scores (selection falls back to index order).
    """
    if not result_path:
        return {}

    result_file = Path(result_path)
    if not result_file.is_file():
        return {}

    text = result_file.read_text()
    try:
        content = yaml.safe_load(text) or {}
        grasps = content.get('grasps', {})
        return {
            grasp_id: float(grasp_data.get('confidence'))
            for grasp_id, grasp_data in grasps.items()
            if 'confidence' in grasp_data
        }
    except yaml.YAMLError:
        import re
        scores = {}
        pattern = re.compile(
            r'^\s*(grasp_\d+):\s*\n\s*confidence:\s*([0-9eE.+-]+)',
            re.MULTILINE,
        )
        for match in pattern.finditer(text):
            scores[match.group(1)] = float(match.group(2))
        return scores


def collect_grasp_candidates(
        node,
        tf_buffer,
        target_frame,
        object_frame,
        grasp_frame_prefix,
        max_grasps,
        max_consecutive_misses,
        tf_lookup_timeout_sec,
        score_map=None):
    """Collect contiguous grasp TF frames published by GraspGen."""
    candidates = []
    misses = 0
    timeout = Duration(seconds=float(tf_lookup_timeout_sec))
    # NOTE: ``target_frame`` (typically "world") must already be connected
    # to the grasp frame via the ``object`` TF before calling this helper.
    # Callers use ``wait_for_transform`` against ``target_frame <- object``
    # to avoid falling into a partial tree where some grasps resolve via
    # ``world`` and others via ``object`` — that mismatch would label
    # object-frame coordinates as world and send bogus goals to cuMotion.
    del object_frame
    for grasp_index in range(int(max_grasps)):
        grasp_id = f'{grasp_frame_prefix}{grasp_index}'
        try:
            transform = tf_buffer.lookup_transform(
                target_frame, grasp_id, Time(), timeout=timeout)
        except TransformException:
            misses += 1
            if candidates and misses >= int(max_consecutive_misses):
                break
            continue

        candidates.append(GraspCandidate(
            frame_id=target_frame,
            grasp_id=grasp_id,
            pose=transform_to_pose(transform),
            confidence=(score_map or {}).get(grasp_id),
        ))
        misses = 0

    node.get_logger().info(
        f'Collected {len(candidates)} grasp TF frames from prefix '
        f'"{grasp_frame_prefix}".')
    return candidates
