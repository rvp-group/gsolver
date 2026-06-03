"""G2O file parser for pose graph data.
"""

from typing import List, Tuple
import numpy as np

from .data_types import Pose, SE3Edge, SE3Prior, LandmarkMeas
from .logger import LOG_INFO


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix."""
    # Normalize quaternion
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    R = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )
    return R


def _parse_upper_triangular_covariance(tokens: List[str], start_idx: int) -> np.ndarray:
    """Parse upper triangular covariance matrix (6x6 symmetric) from tokens."""
    cov = np.zeros((6, 6))
    idx = start_idx
    for row in range(6):
        for col in range(row, 6):
            cov[row, col] = float(tokens[idx])
            cov[col, row] = cov[row, col]  # Symmetric
            idx += 1
    return cov


def _parse_pose_from_tokens(tokens: List[str], start_idx: int = 2) -> Tuple[np.ndarray, np.ndarray]:
    """
    Parse pose and covariance from tokens.
    Format: ... tx ty tz qx qy qz qw [upper_triangular_cov_21_values]
    """
    tx = float(tokens[start_idx])
    ty = float(tokens[start_idx + 1])
    tz = float(tokens[start_idx + 2])
    qx = float(tokens[start_idx + 3])
    qy = float(tokens[start_idx + 4])
    qz = float(tokens[start_idx + 5])
    qw = float(tokens[start_idx + 6])

    # Create transformation matrix from quaternion and translation
    pose = np.eye(4)
    pose[:3, 3] = [tx, ty, tz]
    pose[:3, :3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)

    # Parse covariance
    cov = _parse_upper_triangular_covariance(tokens, start_idx + 7)

    return pose, cov


def parse_g2o_file(filename: str) -> Tuple[List[Pose], List[SE3Edge], List[SE3Prior], List[LandmarkMeas]]:
    """
    Parse a G2O file and extract vertices, edges, priors, and landmarks.

    G2O format:
    - VERTEX_SE3 id tx ty tz qx qy qz qw [21 upper triangular cov values]
    - EDGE_POSE_POSE_SE3 id_from id_to tx ty tz qx qy qz qw [21 upper triangular cov values]
    - EDGE_PRIOR_SE3 id tx ty tz qx qy qz qw [21 upper triangular cov values]
    - EDGE_POSE_LANDMARK_SE3 pose_id landmark_id tx ty tz qx qy qz qw [21 upper triangular cov values]

    Args:
        filename: Path to the G2O file.

    Returns:
        Tuple of (vertices, edges, priors, landmarks) lists.
    """
    vertices = []
    edges = []
    priors = []
    landmarks = []

    with open(filename, "r") as f:
        for line in f:
            tokens = line.strip().split()
            if not tokens:
                continue

            if tokens[0] == "VERTEX_SE3":
                timestamp = float(tokens[1])
                pose, cov = _parse_pose_from_tokens(tokens, start_idx=2)
                vertices.append(Pose(timestamp=timestamp, pose=pose, covariance=cov))

            elif tokens[0] == "EDGE_POSE_POSE_SE3":
                timestamp_from = float(tokens[1])
                timestamp_to = float(tokens[2])
                transformation, cov = _parse_pose_from_tokens(tokens, start_idx=3)
                edges.append(
                    SE3Edge(
                        timestamp_from=timestamp_from,
                        timestamp_to=timestamp_to,
                        transformation=transformation,
                        covariance=cov,
                    )
                )

            elif tokens[0] == "EDGE_PRIOR_SE3":
                timestamp = float(tokens[1])
                pose, cov = _parse_pose_from_tokens(tokens, start_idx=2)
                priors.append(SE3Prior(timestamp=timestamp, pose=pose, covariance=cov))

            elif tokens[0] == "EDGE_POSE_LANDMARK_SE3":
                timestamp = float(tokens[1])
                landmark_id = int(tokens[2])
                measurement, cov = _parse_pose_from_tokens(tokens, start_idx=3)
                landmarks.append(
                    LandmarkMeas(
                        timestamp=timestamp,
                        landmark_id=landmark_id,
                        measurement=measurement,
                        covariance=cov,
                    )
                )

    LOG_INFO("Parsed {} vertices, {} edges, {} SE3 priors, {} landmarks", len(vertices), len(edges), len(priors), len(landmarks))
    return vertices, edges, priors, landmarks
