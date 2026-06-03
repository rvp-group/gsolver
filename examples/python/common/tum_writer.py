"""TUM trajectory format writer.
"""

from typing import List, Tuple
import numpy as np

from .data_types import Pose
from .logger import LOG_INFO


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to quaternion [qx, qy, qz, qw]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return qx, qy, qz, qw


def write_tum(filename: str, poses: List[Pose]) -> None:
    """
    Write poses to a TUM format file.

    TUM format: timestamp tx ty tz qx qy qz qw

    Args:
        filename: Output file path.
        poses: List of Pose objects to write.
    """
    # Sort by timestamp
    sorted_poses = sorted(poses, key=lambda p: p.timestamp)

    with open(filename, "w") as f:
        for pose in sorted_poses:
            tx, ty, tz = pose.pose[:3, 3]
            qx, qy, qz, qw = rotation_matrix_to_quaternion(pose.pose[:3, :3])
            f.write(f"{pose.timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")

    LOG_INFO("Written {} poses to {}", len(poses), filename)


def write_tum_from_factor_graph(fg, filename: str) -> None:
    """
    Write optimized poses from a factor graph to TUM format.

    Extracts poses from variables named "POSE_<timestamp>".
    Mirrors the C++ writeTUM(FactorGraph&, std::ofstream&) function.

    Args:
        fg: FactorGraph containing optimized poses.
        filename: Output file path.
    """
    poses = []

    # Use get_pose_variables() which returns sorted pose variable nodes
    # This already filters for POSE_ variables and sorts by timestamp
    pose_vars = fg.get_pose_variables()

    for var in pose_vars:
        # Use the timestamp attribute directly (all pose variables have it)
        poses.append(Pose(
            timestamp=var.timestamp,
            pose=var.mu.pose,
            covariance=np.eye(6),
        ))

    write_tum(filename, poses)


def write_tum_from_matrices(
    filename: str, timestamps: List[float], poses: List[np.ndarray]
) -> None:
    """
    Write poses to a TUM format file from timestamps and 4x4 matrices.

    Args:
        filename: Output file path.
        timestamps: List of timestamps.
        poses: List of 4x4 transformation matrices.
    """
    if len(timestamps) != len(poses):
        raise ValueError("timestamps and poses must have the same length")

    with open(filename, "w") as f:
        for timestamp, pose in zip(timestamps, poses):
            tx, ty, tz = pose[:3, 3]
            qx, qy, qz, qw = rotation_matrix_to_quaternion(pose[:3, :3])
            f.write(f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")

    LOG_INFO("Written {} poses to {}", len(poses), filename)
