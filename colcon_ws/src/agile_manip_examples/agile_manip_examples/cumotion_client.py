"""cuMotion client that requests a real plan and replays the returned trajectory."""

import rclpy
from geometry_msgs.msg import Point
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from agile_manip_examples.cumotion_utils import (
    ARM_JOINT_NAMES,
    GRIPPER_OPEN,
    HOME_ARM_JOINTS,
    HOME_JOINTS,
    JOINT_NAMES,
    build_joint_goal,
    expand_robot_trajectory,
    moveit_error_name,
    grasp_frame_xyz,
    width_to_finger_joint,
)


# Target chosen to put the forward-kinematic tool pose clearly away from
# HOME so the robot visibly swings to the side during playback. cuMotion
# converts joint_constraints into a pose goal via FK and plans pose-space
# with 7-DOF null-space freedom, so joints may land at different values
# that produce the same pose; picking a dramatically different tool pose
# guarantees visible motion regardless.
DEFAULT_TARGET_ARM_JOINTS = [1.2, -0.3, 0.0, -1.0, 0.0, 1.3, 0.0]


class CuMotionClient(Node):
    """Request a joint-space plan from cuMotion and replay it in RViz."""

    def __init__(self):
        super().__init__('cumotion_client')

        self.declare_parameter('move_group_action_name', 'cumotion/move_group')
        self.declare_parameter('planner_group_name', 'arm')
        self.declare_parameter('pipeline_id', 'isaac_ros_cumotion')
        self.declare_parameter('planner_id', 'cuMotion')
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('playback_period_sec', 0.05)
        self.declare_parameter('target_arm_joints', DEFAULT_TARGET_ARM_JOINTS)
        # Gripper is driven separately from cuMotion. Default is fully
        # open (finger_joint = 0). Set ``gripper_finger_joint_target`` to
        # ~0.7 rad to close the fingers, or ``gripper_width_m`` (0.0 ..
        # 0.14 m) to specify a stroke directly (negative = disabled).
        self.declare_parameter('gripper_finger_joint_target', 0.0)
        self.declare_parameter('gripper_width_m', -1.0)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/cumotion/trajectory_markers', 10)
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            self.get_parameter('move_group_action_name').value,
        )

        self.trajectory = []
        self.current_waypoint_index = 0
        self.playback_timer = None

        self.publish_joint_state(HOME_JOINTS)
        self.startup_timer = self.create_timer(1.0, self.request_plan_once)

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

    def request_plan_once(self):
        """Send the cuMotion action goal when the server becomes ready."""
        if not rclpy.ok():
            return
        try:
            server_ready = self.move_group_client.wait_for_server(timeout_sec=0.1)
        except Exception:
            return
        if not server_ready:
            self.get_logger().info('Waiting for cuMotion MoveGroup action server...')
            return

        self.startup_timer.cancel()
        goal_msg = build_joint_goal(
            self.get_parameter('planner_group_name').value,
            self.get_parameter('pipeline_id').value,
            self.get_parameter('planner_id').value,
            HOME_ARM_JOINTS,
            self.get_parameter('target_arm_joints').value,
            self.get_parameter('allowed_planning_time').value,
        )
        self.get_logger().info('Sending real joint-space planning request to cuMotion.')
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_goal_response)

    def handle_goal_response(self, future):
        """Handle goal acceptance and subscribe to the final action result."""
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().error(f'Failed to send cuMotion goal: {error}')
            return
        if not goal_handle.accepted:
            self.get_logger().error('cuMotion rejected the MoveGroup goal.')
            return

        self.get_logger().info('cuMotion accepted the MoveGroup goal.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_plan_result)

    def handle_plan_result(self, future):
        """Convert the action result into a replayable trajectory."""
        try:
            result = future.result().result
        except Exception as error:  # pragma: no cover - ROS callback path
            self.get_logger().error(f'Failed to receive cuMotion result: {error}')
            return
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'cuMotion planning failed with {moveit_error_name(error_code)}.')
            return

        self.trajectory = expand_robot_trajectory(
            result.planned_trajectory,
            gripper_value=self._gripper_value())
        if not self.trajectory:
            self.get_logger().error('cuMotion returned an empty trajectory.')
            return

        self.publish_trajectory_markers()
        self.get_logger().info(
            f'cuMotion returned {len(self.trajectory)} waypoints '
            f'from {HOME_ARM_JOINTS} to {self.get_parameter("target_arm_joints").value}.')
        self.playback_timer = self.create_timer(
            self.get_parameter('playback_period_sec').value,
            self.publish_next_waypoint,
        )

    def publish_next_waypoint(self):
        """Play the trajectory forward once, then hold at the goal."""
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
        """Log whether the final tool pose matches the target's FK pose.

        cuMotion converts joint_constraints into a goal pose via forward
        kinematics and then plans in pose space with 7-DOF null-space
        freedom, so the final joint values can legitimately differ from
        ``target_arm_joints`` while still reaching the same tool pose.
        We therefore compare tool-frame positions rather than raw joint
        angles.
        """
        target_joints = list(self.get_parameter('target_arm_joints').value)
        final_joints = self.trajectory[-1][:len(ARM_JOINT_NAMES)]
        tx, ty, tz = grasp_frame_xyz(target_joints)
        fx, fy, fz = grasp_frame_xyz(final_joints)
        residual = ((tx - fx) ** 2 + (ty - fy) ** 2 + (tz - fz) ** 2) ** 0.5
        if residual < 0.02:
            self.get_logger().info(
                f'Reached target tool pose (position residual '
                f'{residual*1000:.1f} mm).')
        else:
            self.get_logger().warn(
                f'Trajectory end tool pose is {residual*1000:.1f} mm away '
                f'from the target pose (final=({fx:.3f},{fy:.3f},{fz:.3f}), '
                f'target=({tx:.3f},{ty:.3f},{tz:.3f})).')

    def publish_trajectory_markers(self):
        """Publish the TCP path as a LINE_STRIP + endpoint spheres."""
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        path_marker = Marker()
        path_marker.header.frame_id = 'world'
        path_marker.header.stamp = stamp
        path_marker.ns = 'trajectory_path'
        path_marker.id = 0
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

        endpoint_specs = (
            (HOME_JOINTS, (0.0, 1.0, 0.0)),
            (self.trajectory[-1], (1.0, 0.0, 0.0)),
        )
        for sphere_index, (joints, color) in enumerate(endpoint_specs):
            x, y, z = grasp_frame_xyz(joints)
            sphere_marker = Marker()
            sphere_marker.header = path_marker.header
            sphere_marker.ns = 'waypoints'
            sphere_marker.id = sphere_index + 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = x
            sphere_marker.pose.position.y = y
            sphere_marker.pose.position.z = z
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.03
            sphere_marker.scale.y = 0.03
            sphere_marker.scale.z = 0.03
            red, green, blue = color
            sphere_marker.color.r = red
            sphere_marker.color.g = green
            sphere_marker.color.b = blue
            sphere_marker.color.a = 1.0
            marker_array.markers.append(sphere_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CuMotionClient()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
