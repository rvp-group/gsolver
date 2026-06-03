"""
Rerun visualization utilities for pose graph examples.

Mirrors the C++ rerun_utils.h for consistent visualization across languages.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np

try:
    import rerun as rr
    HAS_RERUN = True
except ImportError:
    HAS_RERUN = False
    rr = None

from gsolver import FactorGraph


@dataclass
class VisualizationParams:
    """Visualization parameters for GP interpolation."""
    qc_diag: np.ndarray  # GP prior covariance diagonal (6,)
    hertz: int = 100     # Interpolation frequency


def init_rerun(app_name: str = "gsolver", spawn: bool = True) -> bool:
    """
    Initialize Rerun visualization.

    Args:
        app_name: Name for the Rerun application.
        spawn: Whether to spawn a new Rerun viewer.

    Returns:
        True if Rerun was initialized successfully, False otherwise.
    """
    if not HAS_RERUN:
        return False

    rr.init(app_name, spawn=spawn)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    return True


def visualize_factor_graph(
    fg: FactorGraph,
    entity_name: str = "",
    iteration: int = 0,
    interp_params: Optional[VisualizationParams] = None,
    trajectory_color: Tuple[int, int, int] = (0, 114, 189),
) -> None:
    """
    Visualize a factor graph in Rerun.

    Displays:
    - Trajectory as a continuous line strip (GP-interpolated if params provided)
    - Pose waypoints as black dots
    - Landmarks as red dots (if present)

    Args:
        fg: Factor graph to visualize
        entity_name: Entity name prefix for Rerun logging
        iteration: Current iteration number (used for time sequence)
        interp_params: Optional interpolation params (if None, uses direct waypoint connection)
        trajectory_color: RGB color for trajectory line strip
    """
    if not HAS_RERUN:
        return

    # Set time sequence for proper timeline in Rerun viewer
    rr.set_time_sequence("iteration", iteration)

    # Collect pose waypoints and landmarks from factor graph
    waypoints = []
    landmarks = []

    # Use get_pose_variables() which returns sorted pose variable nodes
    # This avoids issues with accessing var.id
    pose_vars = fg.get_pose_variables()

    for var in pose_vars:
        pose_matrix = var.mu.pose  # 4x4 transformation matrix
        translation = pose_matrix[:3, 3]
        waypoints.append(translation)

    # Get landmarks by checking variable type
    # SE3Point has mu as Vector3d (numpy array with shape (3,))
    # SE3Pose has mu as 4x4 matrix (for landmarks initialized as poses)
    for var in fg.variable_nodes:
        # Skip SE3PoseVel (these are pose variables, already handled above)
        if hasattr(var.mu, 'pose'):
            continue
        
        # Check if it's SE3Point (mu is Vector3d)
        if isinstance(var.mu, np.ndarray) and var.mu.shape == (3,):
            landmarks.append(var.mu.copy())
        # Check if it's SE3Pose (mu is 4x4 matrix) - used for landmarks in charuco
        elif isinstance(var.mu, np.ndarray) and var.mu.shape == (4, 4):
            landmarks.append(var.mu[:3, 3].copy())

    # Compute trajectory line strip
    # TODO: Add GP interpolation support when available in Python bindings
    trajectory_strip = waypoints

    # Log trajectory as continuous line strip
    if trajectory_strip:
        positions = np.array(trajectory_strip)
        rr.log(
            f"{entity_name}trajectory",
            rr.LineStrips3D([positions], colors=[trajectory_color]),
        )

    # Log pose waypoints as black dots
    if waypoints:
        positions = np.array(waypoints)
        rr.log(
            f"{entity_name}poses",
            rr.Points3D(positions, colors=[(0, 0, 0)], radii=[0.02]),
        )

    # Log landmarks as red dots
    if landmarks:
        positions = np.array(landmarks)
        rr.log(
            f"{entity_name}landmarks",
            rr.Points3D(positions, colors=[(255, 0, 0)], radii=[0.01]),
        )


def visualize_poses(
    fg: FactorGraph,
    num_poses: int,
    iteration: Optional[int] = None,
    entity_prefix: str = "",
    point_color: List[int] = [0, 0, 0],
    line_color: List[int] = [0, 114, 189],
    point_radius: float = 0.02,
) -> None:
    """
    Visualize all pose positions in Rerun (backward compatible wrapper).

    Args:
        fg: Factor graph containing the poses.
        num_poses: Number of poses (unused, kept for API compatibility).
        iteration: Current iteration number (for timeline).
        entity_prefix: Rerun entity path prefix.
        point_color: RGB color for pose points.
        line_color: RGB color for trajectory line.
        point_radius: Radius for pose points.
    """
    visualize_factor_graph(
        fg,
        entity_name=entity_prefix,
        iteration=iteration or 0,
        trajectory_color=tuple(line_color),
    )

