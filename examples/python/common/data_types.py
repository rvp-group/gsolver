"""
Data structures for I/O operations.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class Pose:
    """Represents a pose vertex from g2o file."""
    timestamp: float
    pose: np.ndarray  # 4x4 transformation matrix
    covariance: np.ndarray  # 6x6 covariance


@dataclass
class SE3Edge:
    """Represents an SE3 edge (odometry) between two poses."""
    timestamp_from: float
    timestamp_to: float
    transformation: np.ndarray  # 4x4 transformation matrix
    covariance: np.ndarray  # 6x6 covariance


@dataclass
class SE3Prior:
    """Represents a prior factor on a pose."""
    timestamp: float
    pose: np.ndarray  # 4x4 transformation matrix
    covariance: np.ndarray  # 6x6 covariance


@dataclass
class LandmarkMeas:
    """Represents a landmark measurement (pose-to-landmark observation)."""
    timestamp: float  # Timestamp of the observing pose
    landmark_id: int  # Unique ID of the landmark
    measurement: np.ndarray  # 4x4 transformation from pose to landmark
    covariance: np.ndarray  # 6x6 measurement covariance
