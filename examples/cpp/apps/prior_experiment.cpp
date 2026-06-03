/**
 * @file prior_experiment.cpp
 * @brief GP prior experiment with pose priors.
 *
 * This experiment solves a pose graph using only GP motion priors (no odometry):
 * 1. Loads poses and prior constraints from a g2o file
 * 2. Builds a factor graph with GP motion priors and pose priors
 * 3. Solves using GBP
 * 4. Outputs results in TUM format
 *
 * Usage: ./prior_experiment <input.g2o> <output.tum> <trajectory-type> [-visualize]
 *        trajectory-type: printing_room, sphere, helix (from config file)
 */

#include <chrono>
#include <fstream>
#include <omp.h>
#include <optional>
#include <rerun.hpp>
#include <string>

// Project utilities
#include "../common/data_types.h"
#include "../common/experiment_config.h"
#include "../common/factor_graph_builder.h"
#include "../common/g2o_parser.h"
#include "../common/logger.h"
#include "../common/rerun_utils.h"
#include "../common/tum_writer.h"

// gsolver library
#include <gsolver/core/gbp_solver.h>
#include <gsolver/graph/factor_graph.h>
#include <gsolver/maths/geometry3d.h>

using namespace gsolver;
using namespace examples;

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
  LOG_SET_MODULE("PriorExp");

  // ======================= Parse Command Line Arguments =======================
  if (argc < 4) {
    LOG_ERROR("Usage: {} <input.g2o> <output.tum> <trajectory-type> [-visualize]", argv[0]);
    LOG_INFO("  trajectory-type: printing_room, sphere, helix");
    return 1;
  }

  const std::string input_path      = argv[1];
  const std::string output_path     = argv[2];
  const std::string trajectory_type = argv[3];
  const bool visualize              = (argc > 4 && std::string(argv[4]) == "-visualize");

  if (!std::ifstream(input_path)) {
    LOG_ERROR("Cannot open input file: {}", input_path);
    return 1;
  }

  // ======================= Initialize =========================================
  LOG_SECTION("GP Prior Experiment");

  // Load parameters from config file
  auto params = loadExperimentParams(getDefaultConfigPath(), trajectory_type);
  LOG_INFO("Trajectory type: {} - {}", trajectory_type, params.description);
  LOG_DEBUG("qc_diag = [{}, {}, {}, {}, {}, {}]",
            params.qc_diag[0],
            params.qc_diag[1],
            params.qc_diag[2],
            params.qc_diag[3],
            params.qc_diag[4],
            params.qc_diag[5]);
  LOG_DEBUG("num_iterations = {}", params.num_iterations);

  omp_set_num_threads(omp_get_max_threads());
  LOG_DEBUG("Using {} OpenMP threads", omp_get_max_threads());

  // ======================= Load Data ==========================================
  LOG_STEP(1, 4, "Loading data from: {}", input_path);

  std::vector<PoseInit> poses;
  std::vector<OdometryMeas> odometry;
  std::vector<PriorMeas> priors;
  std::vector<LandmarkMeas> landmarks;
  parseG2OFile(input_path, poses, odometry, priors, landmarks);

  LOG_INFO("Found {} poses, {} prior constraints", poses.size(), priors.size());

  // ======================= Build Factor Graph =================================
  LOG_STEP(2, 4, "Building factor graph...");

  FactorGraph factor_graph;
  add_pose_variable_nodes(factor_graph, poses);
  add_prior_factor_nodes(factor_graph, priors);
  add_gp_prior_factor_nodes(factor_graph, params.qc_diag);

  LOG_INFO("Graph ready: {} variables, {} factors", factor_graph.variable_nodes_.size(), factor_graph.factor_nodes_.size());

  // ======================= Optimize with GBP ==================================
  LOG_STEP(3, 4, "Running GBP optimization ({} iterations)...", params.num_iterations);

  GbpSolver solver(SolverScheduleType::SYNCHRONOUS);

  std::optional<rerun::RecordingStream> rec;
  std::optional<VisualizationParams> viz_params;
  if (visualize) {
    rec.emplace("prior_experiment");
    rec->spawn().exit_on_failure();
    viz_params = VisualizationParams{params.qc_diag, 100};
    visualizeFactorGraph(*rec, factor_graph, "", 0, viz_params);
    LOG_DEBUG("Rerun visualization enabled (100Hz interpolation)");
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < params.num_iterations; ++i) {
    solver.performIteration(factor_graph);
    if (visualize && rec) {
      visualizeFactorGraph(*rec, factor_graph, "", i + 1, viz_params);
    }
    LOG_PROGRESS(i + 1, params.num_iterations, "GBP iterations");
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  LOG_INFO("Optimization complete in {}ms", duration.count());

  // ======================= Save Results =======================================
  LOG_STEP(4, 4, "Saving results to: {}", output_path);

  std::ofstream output_file(output_path);
  if (!output_file) {
    LOG_ERROR("Cannot open output file: {}", output_path);
    return 1;
  }
  writeTUM(factor_graph, output_file);

  LOG_SECTION("Done");
  return 0;
}
