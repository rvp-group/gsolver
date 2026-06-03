#!/usr/bin/env python3
"""
Incremental SLAM experiment with Charuco/AprilTag markers.

This experiment demonstrates incremental pose graph optimization:
1. Starts with an anchored first pose at the origin
2. Incrementally adds new poses as landmark measurements arrive
3. Runs GBP iterations after each new pose is added
4. Performs final bundle adjustment after all poses are added

Unlike charuco_experiment_ba.py which uses batch optimization, this
experiment simulates online/incremental SLAM by processing measurements
sequentially and optimizing incrementally.

Usage: python charuco_experiment_islam.py <input.g2o> <output.tum> [-visualize]
"""

import sys
import time
from pathlib import Path

import numpy as np

# Add parent directory to path for imports when running as script
sys.path.insert(0, str(Path(__file__).parent.parent))

# Import common utilities
from common import (
    # Logging
    LOG_SET_MODULE,
    LOG_INFO,
    LOG_DEBUG,
    LOG_ERROR,
    LOG_SECTION,
    LOG_STEP,
    LOG_PROGRESS,
    # I/O
    parse_g2o_file,
    write_tum_from_factor_graph,
    # Config
    load_experiment_params,
    # Factor graph building helpers
    timestamp_to_string,
    pose_id,
    exp_map_se3,
    log_map_se3,
    add_pose_variable_node,
    add_gp_prior_factor_node,
    add_landmark_factor,
    Pose,
    # Visualization
    init_rerun,
    visualize_factor_graph,
    VisualizationParams,
    HAS_RERUN,
)

# Import gsolver
from gsolver import (
    FactorGraph,
    GbpSolver,
    SolverScheduleType,
    SE3PoseVelPrior,
)


def predict_and_add_new_pose(
    factor_graph: FactorGraph,
    new_timestamp: float,
    previous_timestamp: float,
) -> str:
    """
    Predict and add a new pose to the factor graph.

    Uses constant-velocity motion model to predict the next pose based on
    the previous pose's velocity estimate.

    Args:
        factor_graph: The factor graph to add the pose to
        new_timestamp: Timestamp of the new pose
        previous_timestamp: Timestamp of the previous pose

    Returns:
        Variable ID of the new pose
    """
    # Get the previous pose
    prev_var_id = pose_id(previous_timestamp)
    prev_pose_node = factor_graph.get_variable(prev_var_id)

    # Predict new pose using constant-velocity model
    dt = new_timestamp - previous_timestamp
    prev_pose = prev_pose_node.mu.pose
    prev_vel = prev_pose_node.mu.vel

    delta_pose = exp_map_se3(prev_vel * dt)
    predicted_pose = prev_pose @ delta_pose

    # Estimate velocity from predicted motion
    velocity = log_map_se3(delta_pose) / dt if dt > 0 else np.zeros(6)

    # Create and add the new pose variable
    new_pose = Pose(
        timestamp=new_timestamp,
        pose=predicted_pose,
        covariance=np.eye(6),
    )
    sigma = np.eye(12) * 3.0

    return add_pose_variable_node(factor_graph, new_pose, velocity, sigma)


def main():
    LOG_SET_MODULE("CharucoIncrSLAM")

    # ======================= Parse Command Line Arguments =======================
    if len(sys.argv) < 3:
        LOG_ERROR("Usage: {} <input.g2o> <output.tum> [-visualize]", sys.argv[0])
        return 1

    input_path = sys.argv[1]
    output_path = sys.argv[2]
    visualize = len(sys.argv) > 3 and sys.argv[3] == "-visualize"

    if not Path(input_path).exists():
        LOG_ERROR("Cannot open input file: {}", input_path)
        return 1

    # ======================= Initialize =========================================
    LOG_SECTION("Incremental Charuco SLAM")

    # Load experiment parameters from config
    params = load_experiment_params("printing_room")
    LOG_INFO("Loaded config for trajectory: printing_room")

    # Incremental optimization parameters
    iterations_per_pose = 1   # GBP iterations after each new pose
    final_ba_iterations = 10  # Final bundle adjustment iterations

    LOG_DEBUG("iterations_per_pose = {}", iterations_per_pose)
    LOG_DEBUG("final_ba_iterations = {}", final_ba_iterations)

    # ======================= Load Data ==========================================
    LOG_STEP(1, 5, "Loading data from: {}", input_path)

    poses, odometry, priors, landmarks = parse_g2o_file(input_path)

    if not landmarks:
        LOG_ERROR("No landmark measurements found in input file")
        return 1

    LOG_INFO("Found {} landmark observations", len(landmarks))

    # ======================= Initialize Factor Graph ============================
    LOG_STEP(2, 5, "Initializing factor graph with first pose...")

    factor_graph = FactorGraph()
    solver = GbpSolver(SolverScheduleType.SYNCHRONOUS)

    # Initialize first pose at origin
    current_timestamp = landmarks[0].timestamp

    first_pose = Pose(
        timestamp=current_timestamp,
        pose=np.eye(4),
        covariance=np.eye(6),
    )
    zero_velocity = np.zeros(6)
    init_sigma = np.eye(12) * 3.0

    first_var_id = add_pose_variable_node(factor_graph, first_pose, zero_velocity, init_sigma)

    # Add strong prior on first pose to anchor the trajectory (fix gauge freedom)
    prior_id = f"PRIOR_ANCHOR_{timestamp_to_string(current_timestamp)}"
    anchor_prior = SE3PoseVelPrior(prior_id, np.eye(4), np.eye(6) * 1e-12)
    factor_graph.add_factor(anchor_prior, [first_var_id])

    LOG_INFO("First pose anchored at origin (t={})", current_timestamp)

    # Build var_map for landmark factors
    var_map = {current_timestamp: first_var_id}

    # ======================= Initialize Visualization ===========================
    viz_params = None
    viz_iteration = 0
    if visualize and HAS_RERUN:
        init_rerun("charuco_incremental", spawn=True)
        viz_params = VisualizationParams(qc_diag=params["qc_diag"], hertz=100)
        visualize_factor_graph(factor_graph, "", viz_iteration, viz_params)
        viz_iteration += 1
        LOG_DEBUG("Rerun visualization enabled (100Hz interpolation)")

    # ======================= Incremental Optimization ===========================
    LOG_STEP(3, 5, "Running incremental optimization ({} iteration per pose)...", iterations_per_pose)

    start_time = time.time()
    pose_count = 1
    landmark_count = 0

    for landmark in landmarks:
        if landmark.timestamp == current_timestamp:
            # Same timestamp: just add the landmark factor
            add_landmark_factor(factor_graph, landmark, var_map)
            landmark_count += 1

        elif landmark.timestamp > current_timestamp:
            # New timestamp: optimize current graph, then add new pose

            # Run GBP iterations for current pose
            for _ in range(iterations_per_pose):
                solver.perform_iteration(factor_graph)
                if visualize and HAS_RERUN:
                    visualize_factor_graph(factor_graph, "", viz_iteration, viz_params)
                    viz_iteration += 1

            # Predict and add new pose
            new_var_id = predict_and_add_new_pose(factor_graph, landmark.timestamp, current_timestamp)
            var_map[landmark.timestamp] = new_var_id

            # Add GP motion prior between consecutive poses
            var_from = var_map[current_timestamp]
            var_to = var_map[landmark.timestamp]
            add_gp_prior_factor_node(
                factor_graph,
                var_from,
                var_to,
                current_timestamp,
                landmark.timestamp,
                params["qc_diag"],
            )

            # Add this landmark factor to the new pose
            add_landmark_factor(factor_graph, landmark, var_map)

            current_timestamp = landmark.timestamp
            pose_count += 1
            landmark_count += 1

            # Progress update every 10 poses
            if pose_count % 10 == 0:
                LOG_DEBUG("Added pose {} (t={})", pose_count, current_timestamp)

        else:
            LOG_ERROR("Landmarks are not sorted by timestamp (t={} < current={})",
                      landmark.timestamp, current_timestamp)
            return 1

    LOG_INFO("Incremental phase complete: {} poses, {} landmarks", pose_count, landmark_count)

    # ======================= Final Bundle Adjustment ============================
    LOG_STEP(4, 5, "Running final bundle adjustment ({} iterations)...", final_ba_iterations)

    for i in range(final_ba_iterations):
        solver.perform_iteration(factor_graph)
        if visualize and HAS_RERUN:
            visualize_factor_graph(factor_graph, "", viz_iteration, viz_params)
            viz_iteration += 1
        LOG_PROGRESS(i + 1, final_ba_iterations, "Final BA")

    end_time = time.time()
    duration_ms = int((end_time - start_time) * 1000)

    LOG_INFO("Optimization complete in {}ms", duration_ms)
    LOG_INFO("Final graph: {} variables, {} factors",
             factor_graph.num_variables, factor_graph.num_factors)

    # ======================= Save Results =======================================
    LOG_STEP(5, 5, "Saving results to: {}", output_path)

    write_tum_from_factor_graph(factor_graph, output_path)

    LOG_SECTION("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
