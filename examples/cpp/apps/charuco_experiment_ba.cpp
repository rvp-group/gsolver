/**
 * @file charuco_experiment_ba.cpp
 * @brief Bundle adjustment experiment with Charuco/AprilTag markers.
 *
 * This experiment:
 * 1. Loads camera poses and AprilTag detections from a g2o file
 * 2. Optionally perturbs initial guesses with noise
 * 3. Builds a factor graph with GP motion priors and landmark factors
 * 4. Performs bundle adjustment using Gaussian Belief Propagation (GBP)
 *
 * Usage: ./charuco_experiment_ba <input.g2o> <output.tum> <noise_level> [-visualize]
 */

#include <chrono>
#include <fstream>
#include <omp.h>
#include <optional>
#include <random>
#include <rerun.hpp>
#include <string>

// Project utilities
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
// Helper Functions
// ============================================================================

/**
 * @brief Perturb poses with Gaussian noise for testing robustness.
 */
void perturbPoses(std::vector<PoseInit>& poses, double noise_std, int seed = 42) {
  if (noise_std == 0.0)
    return;

  std::default_random_engine gen(seed);
  std::normal_distribution<double> trans_noise(0.0, noise_std);
  std::normal_distribution<double> rot_noise(0.0, noise_std * 0.1);

  for (auto& pose : poses) {
    Eigen::Vector<double, 6> delta;
    delta << trans_noise(gen), trans_noise(gen), trans_noise(gen), rot_noise(gen), rot_noise(gen), rot_noise(gen);
    pose.pose = pose.pose * expMapSE3(delta);
    pose.covariance += Eigen::Matrix<double, 6, 6>::Identity() * (1.0 / noise_std);
  }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
  LOG_SET_MODULE("CharucoBA");

  // ======================= Parse Command Line Arguments =======================
  if (argc < 4) {
    LOG_ERROR("Usage: {} <input.g2o> <output.tum> <noise_level> [-visualize]", argv[0]);
    return 1;
  }

  const std::string input_path  = argv[1];
  const std::string output_path = argv[2];
  const double noise_level      = std::stod(argv[3]);
  const bool visualize          = (argc > 4 && std::string(argv[4]) == "-visualize");

  if (!std::ifstream(input_path)) {
    LOG_ERROR("Cannot open input file: {}", input_path);
    return 1;
  }

  // ======================= Initialize =========================================
  LOG_SECTION("Charuco BA Experiment");
  omp_set_num_threads(omp_get_max_threads());
  LOG_DEBUG("Using {} OpenMP threads", omp_get_max_threads());

  auto params = loadExperimentParams(getDefaultConfigPath(), "printing_room");
  LOG_INFO("Loaded config for trajectory: printing_room");
  LOG_DEBUG("qc_diag = [{}, {}, {}, {}, {}, {}]",
            params.qc_diag[0],
            params.qc_diag[1],
            params.qc_diag[2],
            params.qc_diag[3],
            params.qc_diag[4],
            params.qc_diag[5]);
  LOG_DEBUG("num_iterations = {}", params.num_iterations);

  // ======================= Load Data ==========================================
  LOG_STEP(1, 4, "Loading data from: {}", input_path);

  std::vector<PoseInit> poses;
  std::vector<OdometryMeas> odometry;
  std::vector<PriorMeas> priors;
  std::vector<LandmarkMeas> landmarks;
  parseG2OFile(input_path, poses, odometry, priors, landmarks);

  LOG_INFO("Found {} poses, {} landmark observations", poses.size(), landmarks.size());

  // ======================= Build Factor Graph =================================
  LOG_STEP(2, 4, "Building factor graph...");

  // Optionally perturb initial guesses to test robustness
  if (noise_level > 0.0) {
    LOG_DEBUG("Perturbing poses with noise_std = {}", noise_level);
  }
  perturbPoses(poses, noise_level);

  // Construct the factor graph:
  //   - Pose variables with velocity (SE3PoseVel)
  //   - GP motion priors between consecutive poses
  //   - Landmark factors from AprilTag detections
  //   - Anchor prior on first pose to fix gauge freedom
  FactorGraph factor_graph;
  add_pose_variable_nodes(factor_graph, poses);
  add_gp_prior_factor_nodes(factor_graph, params.qc_diag);
  add_landmark_factors(factor_graph, landmarks);
  fix_first_pose(factor_graph);

  LOG_INFO("Graph ready: {} variables, {} factors", factor_graph.variable_nodes_.size(), factor_graph.factor_nodes_.size());

  // ======================= Optimize with GBP ==================================
  LOG_STEP(3, 4, "Running GBP optimization ({} iterations)...", params.num_iterations);

  GbpSolver solver(SolverScheduleType::SYNCHRONOUS);

  // Initialize visualization (Rerun) - only if requested
  std::optional<rerun::RecordingStream> rec;
  std::optional<VisualizationParams> viz_params;
  if (visualize) {
    rec.emplace("charuco_ba_experiment");
    rec->spawn().exit_on_failure();
    viz_params = VisualizationParams{params.qc_diag, 100};
    visualizeFactorGraph(*rec, factor_graph, "", 0, viz_params);
  }

  // Run iterative optimization
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
