"""
examples.python.common - Common utilities for Python examples.

This module provides:
- Data types (Pose, SE3Edge, SE3Prior)
- G2O file parsing
- TUM trajectory writing
- Rerun visualization helpers
- Experiment configuration loading (shared with C++)
- Factor graph building utilities
"""

# Data types
from .data_types import Pose, SE3Edge, SE3Prior, LandmarkMeas

# File I/O
from .g2o_parser import parse_g2o_file
from .tum_writer import (
    write_tum,
    write_tum_from_matrices,
    write_tum_from_factor_graph,
    rotation_matrix_to_quaternion,
)

# Visualization
from .rerun_utils import (
    visualize_poses,
    visualize_factor_graph,
    init_rerun,
    HAS_RERUN,
    VisualizationParams,
)

# Logging (mirrors C++ logger.h)
from .logger import (
    Logger,
    LogLevel,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_FATAL,
    LOG_PROGRESS,
    LOG_SECTION,
    LOG_STEP,
    LOG_SET_LEVEL,
    LOG_SET_MODULE,
    LOG_SET_COLOR,
)

# Configuration (loads from shared examples/config/)
from .experiment_config import (
    load_experiment_params,
    get_qc_diag,
    get_num_iterations,
    CONFIG_DIR,
    EXPERIMENT_PARAMS_FILE,
)

# Factor graph building
from .factor_graph_builder import (
    # Helper functions
    timestamp_to_string,
    pose_id,
    landmark_id,
    exp_map_se3,
    log_map_se3,
    compute_Qd_from_Qc,
    create_se3_posevel_value,
    # Variable nodes
    add_pose_variable_node,
    add_pose_variable_nodes,
    # Factor nodes
    add_odometry_factor_nodes,
    add_gp_prior_factor_node,
    add_gp_prior_factor_nodes,
    add_prior_factor_nodes,
    add_landmark_factor,
    add_landmark_factors,
    # Utilities
    fix_first_pose,
    predict_pose_from_velocity,
    perturb_poses,
)

__all__ = [
    # Data types
    "Pose",
    "SE3Edge",
    "SE3Prior",
    "LandmarkMeas",
    # File I/O
    "parse_g2o_file",
    "write_tum",
    "write_tum_from_matrices",
    "write_tum_from_factor_graph",
    "rotation_matrix_to_quaternion",
    # Visualization
    "visualize_poses",
    "visualize_factor_graph",
    "init_rerun",
    "HAS_RERUN",
    "VisualizationParams",
    # Logging
    "Logger",
    "LogLevel",
    "LOG_DEBUG",
    "LOG_INFO",
    "LOG_WARN",
    "LOG_ERROR",
    "LOG_FATAL",
    "LOG_PROGRESS",
    "LOG_SECTION",
    "LOG_STEP",
    "LOG_SET_LEVEL",
    "LOG_SET_MODULE",
    "LOG_SET_COLOR",
    # Configuration
    "load_experiment_params",
    "get_qc_diag",
    "get_num_iterations",
    "CONFIG_DIR",
    "EXPERIMENT_PARAMS_FILE",
    # Factor graph building - helpers
    "timestamp_to_string",
    "pose_id",
    "landmark_id",
    "exp_map_se3",
    "log_map_se3",
    "compute_Qd_from_Qc",
    "create_se3_posevel_value",
    # Factor graph building - variables
    "add_pose_variable_node",
    "add_pose_variable_nodes",
    # Factor graph building - factors
    "add_odometry_factor_nodes",
    "add_gp_prior_factor_node",
    "add_gp_prior_factor_nodes",
    "add_prior_factor_nodes",
    "add_landmark_factor",
    "add_landmark_factors",
    # Factor graph building - utilities
    "fix_first_pose",
    "predict_pose_from_velocity",
    "perturb_poses",
]
