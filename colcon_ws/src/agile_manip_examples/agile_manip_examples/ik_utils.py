"""Forward kinematics for iiwa14 + Robotiq 2F-140 used by the visualization helpers.

Implements the kinematics of the LBR iiwa 14 R820 with the Robotiq
2F-140 gripper directly from the URDF parameters used by
``agile_manip_description/urdf/iiwa14_robotiq_2f_140.urdf.xacro`` so the
grasp-frame position used in RViz stays consistent with the TF tree.
"""

import numpy as np


# Kinematic parameters of the seven iiwa14 revolute joints, taken
# verbatim from the URDF. Each entry is ``(xyz_origin, rpy_origin)``;
# the joint axis is the local +Z axis, so the joint angle ``q`` is
# applied as a rotation around Z on top of the ``rpy`` rotation.
IIWA_JOINTS = [
    ((0.0, 0.0, 0.1575), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.2025), (np.pi / 2, 0.0, np.pi)),
    ((0.0, 0.2045, 0.0), (np.pi / 2, 0.0, np.pi)),
    ((0.0, 0.0, 0.2155), (np.pi / 2, 0.0, 0.0)),
    ((0.0, 0.1845, 0.0), (-np.pi / 2, np.pi, 0.0)),
    ((0.0, 0.0, 0.2155), (np.pi / 2, 0.0, 0.0)),
    ((0.0, 0.081, 0.0), (-np.pi / 2, np.pi, 0.0)),
]

# Offset from iiwa_link_7 to grasp_frame along the flange's local +Z
# axis. Must match the ``iiwa_to_grasp_frame`` joint in the URDF
# (``agile_manip_description/urdf/iiwa14_robotiq_2f_140.urdf.xacro``);
# drift here would desync our FK residual logs from the TF tree.
GRASP_FRAME_OFFSET_Z = 0.045

def _rot_x(angle):
    """Rotation matrix about the X axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(angle):
    """Rotation matrix about the Y axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(angle):
    """Rotation matrix about the Z axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rpy_matrix(roll, pitch, yaw):
    """URDF roll-pitch-yaw (ZYX-fixed) rotation matrix."""
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _joint_transform(xyz, rpy, q):
    """4x4 homogeneous transform for a revolute joint.

    The transform is the origin given by ``xyz`` / ``rpy`` followed by
    a rotation of ``q`` radians about the local Z axis.
    """
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy) @ _rot_z(q)
    T[:3, 3] = xyz
    return T


def iiwa_link7_fk(joints):
    """Forward kinematics: joint angles -> ``iiwa_link_7`` (4x4 matrix)."""
    T = np.eye(4)
    for i, (xyz, rpy) in enumerate(IIWA_JOINTS):
        T = T @ _joint_transform(xyz, rpy, joints[i])
    return T


def iiwa_grasp_frame_fk(joints):
    """Forward kinematics: joint angles -> ``grasp_frame`` pose.

    This matches the GraspGen grasp-pose reference (the back of the
    gripper mesh), so residuals reported against GraspGen grasps use
    the same link cuMotion was asked to plan to.
    """
    T = iiwa_link7_fk(joints)
    gb = np.eye(4)
    gb[:3, 3] = (0.0, 0.0, GRASP_FRAME_OFFSET_Z)
    return T @ gb


