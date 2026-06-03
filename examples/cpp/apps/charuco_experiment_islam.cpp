/**
 * @file charuco_experiment_islam.cpp
 * @brief Incremental SLAM experiment with Charuco/AprilTag markers.
 *
 * This experiment demonstrates incremental pose graph optimization:
 * 1. Starts with an anchored first pose at the origin
 * 2. Incrementally adds new poses as landmark measurements arrive
 * 3. Runs GBP iterations after each new pose is added
 * 4. Performs final bundle adjustment after all poses are added
 *
 * Unlike charuco_experiment_ba.cpp which uses batch optimization, this
 * experiment simulates online/incremental SLAM by processing measurements
 * sequentially and optimizing incrementally.
 *
 * Usage: ./charuco_experiment_islam <input.g2o> <output.tum> [-visualize]
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
#include <gsolver/graph/types/factors/se3_posevel_prior.h>
#include <gsolver/maths/geometry3d.h>

using namespace gsolver;
using namespace examples;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Predict and add a new pose to the factor graph.
 *
 * Uses constant-velocity motion model to predict the next pose based on
 * the previous pose's velocity estimate. This is called when a landmark
 * measurement arrives at a new timestamp.
 *
 * @param factor_graph The factor graph to add the pose to
 * @param new_timestamp Timestamp of the new pose
 * @param previous_timestamp Timestamp of the previous pose
 */
void predictAndAddNewPose(FactorGraph& factor_graph, double new_timestamp, double previous_timestamp) {
  // Get the most recent pose
  auto pose_nodes                       = factor_graph.getPoseVariableNodes();
  std::shared_ptr<SE3PoseVel> prev_pose = std::dynamic_pointer_cast<SE3PoseVel>(pose_nodes.back());

  // Predict new pose using constant-velocity model
  double dt                        = new_timestamp - previous_timestamp;
  Eigen::Isometry3d delta_pose     = expMapSE3(prev_pose->mu_.vel * dt);
  Eigen::Isometry3d predicted_pose = prev_pose->mu_.pose * delta_pose;

  // Estimate velocity from predicted motion
  Eigen::Vector<double, 6> velocity = logMapSE3(delta_pose) / dt;

  // Create and add the new pose variable
  PoseInit new_pose;
  new_pose.timestamp  = new_timestamp;
  new_pose.pose       = predicted_pose;
  new_pose.covariance = Eigen::Matrix<double, 6, 6>::Identity();

  Eigen::Matrix<double, 12, 12> sigma = Eigen::Matrix<double, 12, 12>::Identity() * 3.0;
  add_pose_variable_node(factor_graph, new_pose, velocity, sigma);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
  LOG_SET_MODULE("CharucoIncrSLAM");

  // ======================= Parse Command Line Arguments =======================
  if (argc < 3) {
    LOG_ERROR("Usage: {} <input.g2o> <output.tum> [-visualize]", argv[0]);
    return 1;
  }

  const std::string input_path  = argv[1];
  const std::string output_path = argv[2];
  const bool visualize          = (argc > 3 && std::string(argv[3]) == "-visualize");

  if (!std::ifstream(input_path)) {
    LOG_ERROR("Cannot open input file: {}", input_path);
    return 1;
  }

  // ======================= Initialize =========================================
  LOG_SECTION("Incremental Charuco SLAM");
  omp_set_num_threads(omp_get_max_threads());
  LOG_DEBUG("Using {} OpenMP threads", omp_get_max_threads());

  // Load experiment parameters from config
  auto params = loadExperimentParams(getDefaultConfigPath(), "printing_room");
  LOG_INFO("Loaded config for trajectory: printing_room");

  // Incremental optimization parameters
  const int iterations_per_pose = 1;  // GBP iterations after each new pose
  const int final_ba_iterations = 10; // Final bundle adjustment iterations

  LOG_DEBUG("iterations_per_pose = {}", iterations_per_pose);
  LOG_DEBUG("final_ba_iterations = {}", final_ba_iterations);

  // ======================= Load Data ==========================================
  LOG_STEP(1, 5, "Loading data from: {}", input_path);

  std::vector<PoseInit> poses;
  std::vector<OdometryMeas> odometry;
  std::vector<PriorMeas> priors;
  std::vector<LandmarkMeas> landmarks;
  parseG2OFile(input_path, poses, odometry, priors, landmarks);

  if (landmarks.empty()) {
    LOG_ERROR("No landmark measurements found in input file");
    return 1;
  }

  LOG_INFO("Found {} landmark observations", landmarks.size());

  // ======================= Initialize Factor Graph ============================
  LOG_STEP(2, 5, "Initializing factor graph with first pose...");

  FactorGraph factor_graph;
  GbpSolver solver(SolverScheduleType::SYNCHRONOUS);

  // Initialize first pose at origin
  double current_timestamp = landmarks[0].timestamp;

  PoseInit first_pose;
  first_pose.timestamp  = current_timestamp;
  first_pose.pose       = Eigen::Isometry3d::Identity();
  first_pose.covariance = Eigen::Matrix<double, 6, 6>::Identity();

  Eigen::Vector<double, 6> zero_velocity   = Eigen::Vector<double, 6>::Zero();
  Eigen::Matrix<double, 12, 12> init_sigma = Eigen::Matrix<double, 12, 12>::Identity() * 3.0;
  add_pose_variable_node(factor_graph, first_pose, zero_velocity, init_sigma);

  // Add strong prior on first pose to anchor the trajectory (fix gauge freedom)
  std::string prior_id = "PRIOR_ANCHOR_" + timestampToString(current_timestamp);
  auto anchor_prior =
    std::make_shared<SE3PoseVelPrior>(prior_id, Eigen::Isometry3d::Identity(), Eigen::Matrix<double, 6, 6>::Identity() * 1e-12);
  factor_graph.addFactorNode(anchor_prior, {factor_graph.variable_nodes_[0]->id_});

  LOG_INFO("First pose anchored at origin (t={})", current_timestamp);

  // ======================= Initialize Visualization ===========================
  std::optional<rerun::RecordingStream> rec;
  std::optional<VisualizationParams> viz_params;
  int viz_iteration = 0;
  if (visualize) {
    rec.emplace("charuco_incremental");
    rec->spawn().exit_on_failure();
    viz_params = VisualizationParams{params.qc_diag, 100};
    visualizeFactorGraph(*rec, factor_graph, "", viz_iteration++, viz_params);
    LOG_DEBUG("Rerun visualization enabled (100Hz interpolation)");
  }

  // ======================= Incremental Optimization ===========================
  LOG_STEP(3, 5, "Running incremental optimization ({} iteration per pose)...", iterations_per_pose);

  auto start_time    = std::chrono::high_resolution_clock::now();
  int pose_count     = 1;
  int landmark_count = 0;

  for (const auto& landmark : landmarks) {
    if (landmark.timestamp == current_timestamp) {
      // Same timestamp: just add the landmark factor
      add_landmark_factor(factor_graph, landmark);
      landmark_count++;

    } else if (landmark.timestamp > current_timestamp) {
      // New timestamp: optimize current graph, then add new pose

      // Run GBP iterations for current pose
      for (int iter = 0; iter < iterations_per_pose; ++iter) {
        solver.performIteration(factor_graph);
        if (visualize && rec) {
          visualizeFactorGraph(*rec, factor_graph, "", viz_iteration++, viz_params);
        }
      }

      // Predict and add new pose
      predictAndAddNewPose(factor_graph, landmark.timestamp, current_timestamp);

      // Add GP motion prior between consecutive poses
      add_gp_prior_factor_node(factor_graph, current_timestamp, landmark.timestamp, params.qc_diag);

      // Add this landmark factor to the new pose
      add_landmark_factor(factor_graph, landmark);

      current_timestamp = landmark.timestamp;
      pose_count++;
      landmark_count++;

      // Progress update every 10 poses
      if (pose_count % 10 == 0) {
        LOG_DEBUG("Added pose {} (t={})", pose_count, current_timestamp);
      }

    } else {
      LOG_ERROR("Landmarks are not sorted by timestamp (t={} < current={})", landmark.timestamp, current_timestamp);
      return 1;
    }
  }

  LOG_INFO("Incremental phase complete: {} poses, {} landmarks", pose_count, landmark_count);

  // ======================= Final Bundle Adjustment ============================
  LOG_STEP(4, 5, "Running final bundle adjustment ({} iterations)...", final_ba_iterations);

  for (int iter = 0; iter < final_ba_iterations; ++iter) {
    solver.performIteration(factor_graph);
    if (visualize && rec) {
      visualizeFactorGraph(*rec, factor_graph, "", viz_iteration++, viz_params);
    }
    LOG_PROGRESS(iter + 1, final_ba_iterations, "Final BA");
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  LOG_INFO("Optimization complete in {}ms", duration.count());
  LOG_INFO("Final graph: {} variables, {} factors", factor_graph.variable_nodes_.size(), factor_graph.factor_nodes_.size());

  // ======================= Save Results =======================================
  LOG_STEP(5, 5, "Saving results to: {}", output_path);

  std::ofstream output_file(output_path);
  if (!output_file) {
    LOG_ERROR("Cannot open output file: {}", output_path);
    return 1;
  }
  writeTUM(factor_graph, output_file);

  LOG_SECTION("Done");
  return 0;
}
