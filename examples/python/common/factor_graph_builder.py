"""
Factor graph builder utilities for Python experiments.

This module provides helper functions to build factor graphs for SLAM/PGO,
mirroring the C++ factor_graph_builder.h:

Variable Nodes:
  - add_pose_variable_node()     : Add a single SE3 pose+velocity variable
  - add_pose_variable_nodes()    : Add all poses from parsed g2o vertices

Factor Nodes:
  - add_odometry_factor_nodes()  : Add relative pose constraints (odometry)
  - add_gp_prior_factor_node()   : Add a single GP motion prior between two poses
  - add_gp_prior_factor_nodes()  : Add GP motion priors between all consecutive poses
  - add_prior_factor_nodes()     : Add absolute pose priors
  - add_landmark_factor()        : Add pose-to-landmark measurement (AprilTag)
  - add_landmark_factors()       : Add all landmark factors from g2o
  - fix_first_pose()             : Anchor the first pose to fix gauge freedom

Note: All functions use timestamp-based variable IDs: "POSE_<timestamp>"
"""

from typing import List, Optional, Dict
import numpy as np

from .data_types import Pose, SE3Edge, SE3Prior, LandmarkMeas

# Import gsolver
from gsolver import (
    FactorGraph,
    SE3PoseVel,
    SE3PoseVelValue,
    SE3PoseVelPrior,
    SE3PoseVelPoseVel,
    SE3PoseVelGP,
    SE3Pose,
    SE3PoseVelPose,
    exp_map_se3 as emse3,
    log_map_se3 as lmse3,
)


# ============================================================================
# Helper Functions
# ============================================================================

def timestamp_to_string(timestamp: float) -> str:
    """Convert timestamp to string with fixed precision for ID generation."""
    return f"{timestamp:.9f}"


def pose_id(timestamp: float) -> str:
    """Generate pose variable ID from timestamp."""
    return f"POSE_{timestamp_to_string(timestamp)}"


def landmark_id(landmark_idx: int) -> str:
    """Generate landmark variable ID from index."""
    return f"LANDMARK_{landmark_idx}"


def exp_map_se3(xi: np.ndarray) -> np.ndarray:
    """Exponential map from se(3) to SE(3) using gsolver.geometry3d binding."""
    return emse3(xi)


def log_map_se3(T: np.ndarray) -> np.ndarray:
    """Logarithmic map from SE(3) to se(3) using gsolver.geometry3d binding."""
    return lmse3(T)


def compute_Qd_from_Qc(Qc: np.ndarray, dt: float) -> np.ndarray:
    """
    Compute discrete-time GP covariance Qd (12x12) from continuous-time Qc (6x6).

    Q_d = | (dt^3/3)*Qc   (dt^2/2)*Qc |
          | (dt^2/2)*Qc      dt*Qc    |
    """
    Qd = np.zeros((12, 12))
    Qd[0:6, 0:6] = (dt**3 / 3.0) * Qc
    Qd[0:6, 6:12] = (dt**2 / 2.0) * Qc
    Qd[6:12, 0:6] = (dt**2 / 2.0) * Qc
    Qd[6:12, 6:12] = dt * Qc
    return Qd


def create_se3_posevel_value(
    pose: np.ndarray, velocity: Optional[np.ndarray] = None
) -> SE3PoseVelValue:
    """Create an SE3PoseVelValue from a 4x4 pose matrix and optional velocity."""
    value = SE3PoseVelValue()
    value.pose = pose.copy()
    value.vel = velocity if velocity is not None else np.zeros(6)
    return value


# ============================================================================
# Variable Nodes
# ============================================================================

def add_pose_variable_node(
    fg: FactorGraph,
    pose: Pose,
    velocity: np.ndarray,
    sigma: np.ndarray,
) -> str:
    """
    Add a single pose variable node to the factor graph.

    Args:
        fg: The factor graph to add the variable to
        pose: The initial pose data (timestamp, SE3 transform, covariance)
        velocity: Initial velocity estimate (6D: linear + angular)
        sigma: Initial covariance (12x12 for pose+velocity)

    Returns:
        The variable ID string
    """
    var_id = pose_id(pose.timestamp)
    mu = create_se3_posevel_value(pose.pose, velocity)
    var = SE3PoseVel(var_id, mu, sigma, pose.timestamp)
    fg.add_variable(var)
    return var_id


def add_pose_variable_nodes(
    fg: FactorGraph,
    poses: List[Pose],
    sigma: Optional[np.ndarray] = None,
) -> Dict[float, str]:
    """
    Add all pose variable nodes from parsed g2o initial poses.

    Estimates initial velocities from finite differences between consecutive poses.

    Args:
        fg: The factor graph to add variables to
        poses: Vector of initial poses from g2o file
        sigma: Initial covariance (12x12), defaults to 3*I

    Returns:
        Dictionary mapping timestamp -> variable ID
    """
    if sigma is None:
        sigma = np.eye(12) * 3.0

    # Sort by timestamp
    sorted_poses = sorted(poses, key=lambda p: p.timestamp)

    # Estimate velocities from pose differences
    velocities = []
    for i in range(len(sorted_poses) - 1):
        dt = sorted_poses[i + 1].timestamp - sorted_poses[i].timestamp
        if dt > 0:
            delta = np.linalg.inv(sorted_poses[i].pose) @ sorted_poses[i + 1].pose
            v = log_map_se3(delta) / dt
        else:
            v = np.zeros(6)
        velocities.append(v)
    velocities.append(velocities[-1] if velocities else np.zeros(6))

    # Add all variables
    var_map = {}
    for i, pose in enumerate(sorted_poses):
        var_id = add_pose_variable_node(fg, pose, velocities[i], sigma)
        var_map[pose.timestamp] = var_id

    return var_map


# ============================================================================
# Factor Nodes - Odometry (Relative Pose Constraints)
# ============================================================================

def add_odometry_factor_nodes(
    fg: FactorGraph,
    edges: List[SE3Edge],
    var_map: Dict[float, str],
) -> int:
    """
    Add odometry factors from parsed g2o edges.

    Args:
        fg: The factor graph to add factors to
        edges: Vector of odometry measurements from g2o file
        var_map: Dictionary mapping timestamp -> variable ID

    Returns:
        Number of factors added
    """
    count = 0
    for edge in edges:
        var_from = var_map.get(edge.timestamp_from)
        var_to = var_map.get(edge.timestamp_to)

        if var_from is None or var_to is None:
            continue

        # Add small regularization to covariance
        cov = edge.covariance + 1e-6 * np.eye(6)

        factor_id = f"ODOM_{timestamp_to_string(edge.timestamp_from)}_{timestamp_to_string(edge.timestamp_to)}"
        factor = SE3PoseVelPoseVel(factor_id, edge.transformation, cov)
        fg.add_factor(factor, [var_from, var_to])
        count += 1

    return count


# ============================================================================
# Factor Nodes - GP Motion Prior
# ============================================================================

def add_gp_prior_factor_node(
    fg: FactorGraph,
    var_from: str,
    var_to: str,
    from_timestamp: float,
    to_timestamp: float,
    Qc_diag: np.ndarray,
) -> bool:
    """
    Add a single GP motion prior factor between two poses.

    Args:
        fg: The factor graph to add the factor to
        var_from: Variable ID of the first pose
        var_to: Variable ID of the second pose
        from_timestamp: Timestamp of the first pose
        to_timestamp: Timestamp of the second pose
        Qc_diag: Diagonal of the continuous-time power spectral density (6D)

    Returns:
        True if factor was added, False otherwise
    """
    dt = to_timestamp - from_timestamp
    if dt <= 0:
        return False

    # Build Qc with minimum values for numerical stability
    Qc_safe = np.maximum(Qc_diag, 1e-4)
    Qc = np.diag(Qc_safe)

    # Compute discrete-time covariance
    Qd = compute_Qd_from_Qc(Qc, dt)

    factor_id = f"GP_{timestamp_to_string(from_timestamp)}_{timestamp_to_string(to_timestamp)}"
    factor = SE3PoseVelGP(factor_id, dt, Qd)
    fg.add_factor(factor, [var_from, var_to])
    return True


def add_gp_prior_factor_nodes(
    fg: FactorGraph,
    poses: List[Pose],
    var_map: Dict[float, str],
    Qc_diag: np.ndarray,
) -> int:
    """
    Add GP motion prior factors between all consecutive poses.

    Args:
        fg: The factor graph (must already contain pose variables)
        poses: List of poses (used for timestamps)
        var_map: Dictionary mapping timestamp -> variable ID
        Qc_diag: Diagonal of the continuous-time power spectral density (6D)

    Returns:
        Number of factors added
    """
    sorted_poses = sorted(poses, key=lambda p: p.timestamp)
    count = 0

    for i in range(len(sorted_poses) - 1):
        p1, p2 = sorted_poses[i], sorted_poses[i + 1]
        var_from = var_map.get(p1.timestamp)
        var_to = var_map.get(p2.timestamp)

        if var_from and var_to:
            if add_gp_prior_factor_node(fg, var_from, var_to, p1.timestamp, p2.timestamp, Qc_diag):
                count += 1

    return count


# ============================================================================
# Factor Nodes - Prior (Absolute Pose Constraints)
# ============================================================================

def add_prior_factor_nodes(
    fg: FactorGraph,
    priors: List[SE3Prior],
    var_map: Dict[float, str],
) -> int:
    """
    Add absolute pose prior factors from parsed g2o priors.

    Args:
        fg: The factor graph to add factors to
        priors: Vector of prior measurements from g2o file
        var_map: Dictionary mapping timestamp -> variable ID

    Returns:
        Number of factors added
    """
    count = 0
    for prior in priors:
        var_name = var_map.get(prior.timestamp)
        if var_name is None:
            continue

        # Add small regularization to covariance
        cov = prior.covariance + 1e-6 * np.eye(6)

        factor_id = f"PRIOR_{timestamp_to_string(prior.timestamp)}"
        factor = SE3PoseVelPrior(factor_id, prior.pose, cov)
        fg.add_factor(factor, [var_name])
        count += 1

    return count


def add_landmark_factor(
    fg: FactorGraph,
    landmark: LandmarkMeas,
    var_map: Dict[float, str],
    sigma: Optional[np.ndarray] = None,
) -> bool:
    """
    Add a single pose-to-landmark factor.

    Creates a factor between a pose and a landmark. If the landmark doesn't
    exist yet, it is initialized from the current pose and measurement.

    Args:
        fg: The factor graph to add to
        landmark: Landmark measurement data (pose timestamp, landmark ID, relative transform)
        var_map: Dictionary mapping timestamp -> variable ID
        sigma: Measurement covariance (6x6), defaults to 3*I

    Returns:
        True if factor was added, False otherwise
    """
    if sigma is None:
        sigma = np.eye(6) * 3.0

    pose_var_id = var_map.get(landmark.timestamp)
    if pose_var_id is None:
        return False

    lm_var_id = landmark_id(landmark.landmark_id)

    # Initialize landmark if it doesn't exist
    if fg.get_variable(lm_var_id) is None:
        pose_node = fg.get_variable(pose_var_id)
        if pose_node is None:
            return False

        # Initialize landmark position from pose and measurement
        lm_pose = pose_node.mu.pose @ landmark.measurement
        landmark_var = SE3Pose(lm_var_id, lm_pose, sigma)
        fg.add_variable(landmark_var)

    # Factor ID: "<timestamp>_<landmark_id>"
    factor_id = f"{timestamp_to_string(landmark.timestamp)}_{landmark.landmark_id}"
    factor = SE3PoseVelPose(factor_id, landmark.measurement, sigma)
    fg.add_factor(factor, [pose_var_id, lm_var_id])
    return True


def add_landmark_factors(
    fg: FactorGraph,
    landmarks: List[LandmarkMeas],
    var_map: Dict[float, str],
    sigma: Optional[np.ndarray] = None,
) -> int:
    """
    Add all landmark factors from parsed g2o landmark measurements.

    Args:
        fg: The factor graph to add factors to
        landmarks: List of landmark measurements from g2o file
        var_map: Dictionary mapping timestamp -> variable ID
        sigma: Measurement covariance (6x6), defaults to 3*I

    Returns:
        Number of factors added
    """
    if sigma is None:
        sigma = np.eye(6) * 3.0

    count = 0
    for lm in landmarks:
        if add_landmark_factor(fg, lm, var_map, sigma):
            count += 1
    return count


# ============================================================================
# Utility Functions
# ============================================================================

def fix_first_pose(
    fg: FactorGraph,
    poses: List[Pose],
    var_map: Dict[float, str],
    pose: Optional[np.ndarray] = None,
    sigma: float = 1e-6,
) -> None:
    """
    Fix the first pose to remove gauge freedom.

    Adds a strong prior on the first pose to anchor the trajectory.

    Args:
        fg: The factor graph (must already contain pose variables)
        poses: List of poses
        var_map: Dictionary mapping timestamp -> variable ID
        pose: The pose to anchor to (defaults to first pose)
        sigma: Prior covariance scalar - smaller = stronger constraint
    """
    first_pose = min(poses, key=lambda p: p.timestamp)
    first_var = var_map.get(first_pose.timestamp)

    if first_var is None:
        return

    anchor_pose = pose if pose is not None else first_pose.pose
    cov = np.eye(6) * sigma

    factor_id = f"PRIOR_ANCHOR_{timestamp_to_string(first_pose.timestamp)}"
    factor = SE3PoseVelPrior(factor_id, anchor_pose, cov)
    fg.add_factor(factor, [first_var])


def predict_pose_from_velocity(
    fg: FactorGraph,
    var_name: str,
    new_timestamp: float,
    prev_timestamp: float,
) -> tuple:
    """
    Predict a new pose using constant-velocity model.

    Args:
        fg: The factor graph containing the previous pose
        var_name: Variable name of the previous pose
        new_timestamp: Timestamp of the new pose
        prev_timestamp: Timestamp of the previous pose

    Returns:
        tuple: (predicted_pose, predicted_velocity)
    """
    prev_var = fg.get_variable(var_name)
    prev_pose = prev_var.mu.pose
    prev_vel = prev_var.mu.vel

    dt = new_timestamp - prev_timestamp
    delta_pose = exp_map_se3(prev_vel * dt)
    predicted_pose = prev_pose @ delta_pose
    velocity = log_map_se3(delta_pose) / dt if dt > 0 else np.zeros(6)

    return predicted_pose, velocity


def perturb_poses(
    poses: List[Pose],
    noise_std: float,
    seed: int = 42,
) -> List[Pose]:
    """
    Perturb poses with Gaussian noise for testing robustness.

    Args:
        poses: List of poses to perturb
        noise_std: Standard deviation of translation noise
        seed: Random seed

    Returns:
        List of perturbed poses
    """
    if noise_std == 0.0:
        return poses

    np.random.seed(seed)
    perturbed = []

    for pose in poses:
        trans_noise = np.random.normal(0.0, noise_std, 3)
        rot_noise = np.random.normal(0.0, noise_std * 0.1, 3)
        delta = np.concatenate([trans_noise, rot_noise])

        delta_T = exp_map_se3(delta)
        perturbed_pose = pose.pose @ delta_T
        perturbed_cov = pose.covariance + np.eye(6) * (1.0 / noise_std)

        perturbed.append(Pose(
            timestamp=pose.timestamp,
            pose=perturbed_pose,
            covariance=perturbed_cov
        ))

    return perturbed
