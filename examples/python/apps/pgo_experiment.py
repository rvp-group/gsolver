#!/usr/bin/env python3
"""
Pose Graph Optimization experiment using Gaussian Belief Propagation.

This experiment demonstrates pose graph optimization (PGO) with GP motion priors:
1. Loads poses and odometry from a g2o file
2. Builds a factor graph with SE3PoseVel variables
3. Adds odometry factors and GP motion priors
4. Solves using GBP
5. Outputs results in TUM format

Usage: python pgo_experiment.py <input.g2o> <output.tum> <trajectory-type> [-visualize]
       trajectory-type: printing_room, sphere, helix (from config file)
"""

import sys
import time
from pathlib import Path

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
    # Factor graph building
    add_pose_variable_nodes,
    add_odometry_factor_nodes,
    add_gp_prior_factor_nodes,
    # Visualization
    init_rerun,
    visualize_factor_graph,
    VisualizationParams,
    HAS_RERUN,
)

# Import gsolver
from gsolver import FactorGraph, GbpSolver, SolverScheduleType


def main():
    LOG_SET_MODULE("PGO")

    # ======================= Parse Command Line Arguments =======================
    if len(sys.argv) < 4:
        LOG_ERROR("Usage: {} <input.g2o> <output.tum> <trajectory-type> [-visualize]", sys.argv[0])
        LOG_INFO("  trajectory-type: printing_room, sphere, helix")
        return 1

    input_path = sys.argv[1]
    output_path = sys.argv[2]
    trajectory_type = sys.argv[3]
    visualize = len(sys.argv) > 4 and sys.argv[4] == "-visualize"

    if not Path(input_path).exists():
        LOG_ERROR("Cannot open input file: {}", input_path)
        return 1

    # ======================= Initialize =========================================
    LOG_SECTION("Pose Graph Optimization")

    # Load parameters from config file
    params = load_experiment_params(trajectory_type)
    LOG_INFO("Trajectory type: {} - {}", trajectory_type, params.get("description", ""))
    LOG_DEBUG("qc_diag = [{}, {}, {}, {}, {}, {}]",
              params["qc_diag"][0], params["qc_diag"][1], params["qc_diag"][2],
              params["qc_diag"][3], params["qc_diag"][4], params["qc_diag"][5])
    LOG_DEBUG("num_iterations = {}", params["num_iterations"])

    # ======================= Load Data ==========================================
    LOG_STEP(1, 4, "Loading data from: {}", input_path)

    poses, odometry, priors, landmarks = parse_g2o_file(input_path)
    LOG_INFO("Found {} poses, {} odometry edges", len(poses), len(odometry))

    # ======================= Build Factor Graph =================================
    LOG_STEP(2, 4, "Building factor graph...")

    factor_graph = FactorGraph()
    var_map = add_pose_variable_nodes(factor_graph, poses)
    add_odometry_factor_nodes(factor_graph, odometry, var_map)
    add_gp_prior_factor_nodes(factor_graph, poses, var_map, Qc_diag=params["qc_diag"])

    LOG_INFO("Graph ready: {} variables, {} factors",
             factor_graph.num_variables, factor_graph.num_factors)

    # ======================= Optimize with GBP ==================================
    LOG_STEP(3, 4, "Running GBP optimization ({} iterations)...", params["num_iterations"])

    solver = GbpSolver(SolverScheduleType.SYNCHRONOUS)

    viz_params = None
    if visualize and HAS_RERUN:
        init_rerun("pgo_experiment", spawn=True)
        viz_params = VisualizationParams(qc_diag=params["qc_diag"], hertz=100)
        visualize_factor_graph(factor_graph, "", 0, viz_params)
        LOG_DEBUG("Rerun visualization enabled (100Hz interpolation)")

    start_time = time.time()
    for i in range(params["num_iterations"]):
        solver.perform_iteration(factor_graph)
        if visualize and HAS_RERUN:
            visualize_factor_graph(factor_graph, "", i + 1, viz_params)
        LOG_PROGRESS(i + 1, params["num_iterations"], "GBP iterations")

    end_time = time.time()
    duration_ms = int((end_time - start_time) * 1000)

    LOG_INFO("Optimization complete in {}ms", duration_ms)

    # ======================= Save Results =======================================
    LOG_STEP(4, 4, "Saving results to: {}", output_path)

    write_tum_from_factor_graph(factor_graph, output_path)

    LOG_SECTION("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
