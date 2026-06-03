#!/usr/bin/env python3
"""
Hyperparameter ablation study for GP motion prior scale.

This experiment evaluates the effect of different GP prior scales on
pose graph optimization:
1. Creates multiple factor graphs with different GP prior scales
   (0.01, 0.1, 1.0, 10.0, 100.0)
2. Solves each graph independently using GBP
3. Outputs results for comparison

Usage: python hyperparam_ablation_experiment.py <input.g2o> <output_folder> <trajectory-type> [-visualize]
       trajectory-type: printing_room, sphere, helix (from config file)
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
    # Factor graph building
    add_pose_variable_nodes,
    add_prior_factor_nodes,
    add_gp_prior_factor_nodes,
    add_odometry_factor_nodes,
    # Visualization
    init_rerun,
    visualize_factor_graph,
    VisualizationParams,
    HAS_RERUN,
)

if HAS_RERUN:
    import rerun as rr

# Import gsolver
from gsolver import FactorGraph, GbpSolver, SolverScheduleType


def main():
    LOG_SET_MODULE("HyperparamAblation")

    # ======================= Parse Command Line Arguments =======================
    if len(sys.argv) < 4:
        LOG_ERROR("Usage: {} <input.g2o> <output_folder> <trajectory-type> [-visualize]", sys.argv[0])
        LOG_INFO("  trajectory-type: printing_room, sphere, helix")
        return 1

    input_path = sys.argv[1]
    output_folder = sys.argv[2]
    trajectory_type = sys.argv[3]
    visualize = len(sys.argv) > 4 and sys.argv[4] == "-visualize"

    if not Path(input_path).exists():
        LOG_ERROR("Cannot open input file: {}", input_path)
        return 1

    # ======================= Initialize =========================================
    LOG_SECTION("Hyperparameter Ablation Study")

    # Load base parameters from config file
    params = load_experiment_params(trajectory_type)
    LOG_INFO("Trajectory type: {} - {}", trajectory_type, params.get("description", ""))
    LOG_DEBUG("Base qc_diag = [{}, {}, {}, {}, {}, {}]",
              params["qc_diag"][0], params["qc_diag"][1], params["qc_diag"][2],
              params["qc_diag"][3], params["qc_diag"][4], params["qc_diag"][5])

    # Define scale factors for ablation
    scales = [0.01, 0.1, 1.0, 10.0, 100.0]
    scale_names = ["scale_001", "scale_01", "scale_1", "scale_10", "scale_100"]

    LOG_INFO("Testing {} different scales: 0.01, 0.1, 1.0, 10.0, 100.0", len(scales))

    # ======================= Load Data ==========================================
    LOG_STEP(1, 4, "Loading data from: {}", input_path)

    poses, odometry, priors, landmarks = parse_g2o_file(input_path)
    LOG_INFO("Found {} poses, {} odometry edges, {} priors", len(poses), len(odometry), len(priors))

    # ======================= Build Factor Graphs ================================
    LOG_STEP(2, 4, "Building {} factor graphs with different scales...", len(scales))

    factor_graphs = []
    var_maps = []

    for i, scale in enumerate(scales):
        fg = FactorGraph()
        var_map = add_pose_variable_nodes(fg, poses)
        add_prior_factor_nodes(fg, priors, var_map)

        # Scale the qc_diag
        scaled_qc_diag = np.array(params["qc_diag"]) * scale
        add_gp_prior_factor_nodes(fg, poses, var_map, scaled_qc_diag)
        add_odometry_factor_nodes(fg, odometry, var_map)

        factor_graphs.append(fg)
        var_maps.append(var_map)

        LOG_DEBUG("Graph {} (scale={}): {} variables, {} factors",
                  scale_names[i], scale, fg.num_variables, fg.num_factors)

    # ======================= Optimize with GBP ==================================
    LOG_STEP(3, 4, "Running GBP optimization ({} iterations per graph)...", params["num_iterations"])

    solver = GbpSolver(SolverScheduleType.SYNCHRONOUS)

    # Optional visualization with different colors per scale
    colors = [
        (255, 0, 0),      # Red
        (0, 255, 0),      # Green
        (0, 0, 255),      # Blue
        (255, 255, 0),    # Yellow
        (255, 0, 255),    # Magenta
    ]

    if visualize and HAS_RERUN:
        init_rerun("hyperparam_ablation", spawn=True)
        LOG_DEBUG("Rerun visualization enabled (no interpolation for performance)")

    start_time = time.time()

    for iter_num in range(params["num_iterations"]):
        for i, fg in enumerate(factor_graphs):
            solver.perform_iteration(fg)
            if visualize and HAS_RERUN:
                rr.set_time_sequence("iteration", iter_num)
                visualize_factor_graph(fg, f"{scale_names[i]}/", iter_num, None, colors[i])
        LOG_PROGRESS(iter_num + 1, params["num_iterations"], "GBP iterations")

    end_time = time.time()
    duration_ms = int((end_time - start_time) * 1000)

    LOG_INFO("All optimizations complete in {}ms", duration_ms)

    # ======================= Save Results =======================================
    LOG_STEP(4, 4, "Saving results to: {}/", output_folder)

    # Create output directory if it doesn't exist
    Path(output_folder).mkdir(parents=True, exist_ok=True)

    for i, fg in enumerate(factor_graphs):
        output_path = Path(output_folder) / f"{scale_names[i]}.tum"
        write_tum_from_factor_graph(fg, str(output_path))
        LOG_DEBUG("Saved: {}", output_path)

    LOG_SECTION("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
