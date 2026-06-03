/**
 * @file hyperparam_ablation_experiment.cpp
 * @brief Hyperparameter ablation study for GP motion prior scale.
 *
 * This experiment evaluates the effect of different GP prior scales on
 * pose graph optimization:
 * 1. Creates multiple factor graphs with different GP prior scales
 *    (0.01, 0.1, 1.0, 10.0, 100.0)
 * 2. Solves each graph independently using GBP
 * 3. Outputs results for comparison
 *
 * Usage: ./hyperparam_ablation_experiment <input.g2o> <output_folder> <trajectory-type> [-visualize]
 *        trajectory-type: printing_room, sphere, helix (from config file)
 */

#include <chrono>
#include <filesystem>
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
  LOG_SET_MODULE("HyperparamAblation");

  // ======================= Parse Command Line Arguments =======================
  if (argc < 4) {
    LOG_ERROR("Usage: {} <input.g2o> <output_folder> <trajectory-type> [-visualize]", argv[0]);
    LOG_INFO("  trajectory-type: printing_room, sphere, helix");
    return 1;
  }

  const std::string input_path      = argv[1];
  const std::string output_folder   = argv[2];
  const std::string trajectory_type = argv[3];
  const bool visualize              = (argc > 4 && std::string(argv[4]) == "-visualize");

  if (!std::ifstream(input_path)) {
    LOG_ERROR("Cannot open input file: {}", input_path);
    return 1;
  }

  // ======================= Initialize =========================================
  LOG_SECTION("Hyperparameter Ablation Study");

  // Load base parameters from config file
  auto params = loadExperimentParams(getDefaultConfigPath(), trajectory_type);
  LOG_INFO("Trajectory type: {} - {}", trajectory_type, params.description);
  LOG_DEBUG("Base qc_diag = [{}, {}, {}, {}, {}, {}]",
            params.qc_diag[0],
            params.qc_diag[1],
            params.qc_diag[2],
            params.qc_diag[3],
            params.qc_diag[4],
            params.qc_diag[5]);

  omp_set_num_threads(omp_get_max_threads());
  LOG_DEBUG("Using {} OpenMP threads", omp_get_max_threads());

  // Define scale factors for ablation
  const std::vector<double> scales           = {0.01, 0.1, 1.0, 10.0, 100.0};
  const std::vector<std::string> scale_names = {"scale_001", "scale_01", "scale_1", "scale_10", "scale_100"};

  LOG_INFO("Testing {} different scales: 0.01, 0.1, 1.0, 10.0, 100.0", scales.size());

  // ======================= Load Data ==========================================
  LOG_STEP(1, 4, "Loading data from: {}", input_path);

  std::vector<PoseInit> poses;
  std::vector<OdometryMeas> odometry;
  std::vector<PriorMeas> priors;
  std::vector<LandmarkMeas> landmarks;
  parseG2OFile(input_path, poses, odometry, priors, landmarks);

  LOG_INFO("Found {} poses, {} odometry edges, {} priors", poses.size(), odometry.size(), priors.size());

  // ======================= Build Factor Graphs ================================
  LOG_STEP(2, 4, "Building {} factor graphs with different scales...", scales.size());

  std::vector<FactorGraph> factor_graphs(scales.size());

  for (size_t i = 0; i < scales.size(); ++i) {
    add_pose_variable_nodes(factor_graphs[i], poses);
    add_prior_factor_nodes(factor_graphs[i], priors);
    add_gp_prior_factor_nodes(factor_graphs[i], params.qc_diag * scales[i]);
    add_odometry_factor_nodes(factor_graphs[i], odometry);
    LOG_DEBUG("Graph {} (scale={}): {} variables, {} factors",
              scale_names[i],
              scales[i],
              factor_graphs[i].variable_nodes_.size(),
              factor_graphs[i].factor_nodes_.size());
  }

  // ======================= Optimize with GBP ==================================
  LOG_STEP(3, 4, "Running GBP optimization ({} iterations per graph)...", params.num_iterations);

  GbpSolver solver(SolverScheduleType::SYNCHRONOUS);

  // Optional visualization with different colors per scale
  std::optional<rerun::RecordingStream> rec;
  std::vector<rerun::Color> colors;
  if (visualize) {
    rec.emplace("hyperparam_ablation");
    rec->spawn().exit_on_failure();
    colors = {rerun::Color(255, 0, 0),    // Red
              rerun::Color(0, 255, 0),    // Green
              rerun::Color(0, 0, 255),    // Blue
              rerun::Color(255, 255, 0),  // Yellow
              rerun::Color(255, 0, 255)}; // Magenta
    LOG_DEBUG("Rerun visualization enabled (no interpolation for performance)");
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  for (int iter = 0; iter < params.num_iterations; ++iter) {
    for (size_t i = 0; i < factor_graphs.size(); ++i) {
      solver.performIteration(factor_graphs[i]);
      if (visualize && rec) {
        visualizeFactorGraph(*rec, factor_graphs[i], scale_names[i] + "/", iter, std::nullopt, colors[i]);
      }
    }
    LOG_PROGRESS(iter + 1, params.num_iterations, "GBP iterations");
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  LOG_INFO("All optimizations complete in {}ms", duration.count());

  // ======================= Save Results =======================================
  LOG_STEP(4, 4, "Saving results to: {}/", output_folder);

  // Create output directory if it doesn't exist
  std::filesystem::create_directories(output_folder);

  for (size_t i = 0; i < factor_graphs.size(); ++i) {
    std::string output_path = output_folder + "/" + scale_names[i] + ".tum";
    std::ofstream output_file(output_path);

    if (!output_file) {
      LOG_ERROR("Cannot open output file: {}", output_path);
      continue;
    }

    writeTUM(factor_graphs[i], output_file);
    LOG_DEBUG("Saved: {}", output_path);
  }

  LOG_SECTION("Done");
  return 0;
}
