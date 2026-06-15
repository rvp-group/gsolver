/**
 * @file stereo_visual.cpp
 * @brief Stereo visual SLAM with SE3PoseVel variables and GP motion priors.
 *
 * Supports single-robot and multi-robot modes. In multi-robot mode each robot
 * gets an independent factor graph covering its portion of the trajectory, and
 * shared landmarks are linked by equality factors across graphs.
 * After optimization the trajectory is GP-interpolated at ground-truth timestamps
 * and written in TUM format.
 *
 * Usage:
 *   ./stereo_visual <input.g2o> <gt.tum> <output.tum> <trajectory-type>
 *                  [num_robots] [-visualize]
 *   trajectory-type: kitti_00..kitti_10, euroc_MH01..euroc_V203  (from config file)
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <omp.h>
#include <optional>
#include <rerun.hpp>
#include <set>
#include <string>
#include <vector>

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
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/gp_interpolation.h>
#include <gsolver/maths/geometry3d.h>

using namespace gsolver;
using namespace examples;

// ============================================================================
// Ground-truth TUM reader (timestamp tx ty tz qx qy qz qw)
// ============================================================================

static std::vector<double> loadGTTimestamps(const std::string& path) {
  std::vector<double> timestamps;
  std::ifstream file(path);
  if (!file.is_open()) {
    LOG_ERROR("Cannot open ground-truth file: {}", path);
    return timestamps;
  }
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream ss(line);
    double t;
    if (ss >> t)
      timestamps.push_back(t);
  }
  return timestamps;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
  LOG_SET_MODULE("StereoVisual");

  // ======================= Parse Command Line Arguments =======================
  if (argc < 5) {
    LOG_ERROR("Usage: {} <input.g2o> <gt.tum> <output.tum> <trajectory-type> [num_robots] [-visualize]",
              argv[0]);
    LOG_INFO("  trajectory-type: kitti_00..kitti_10, euroc_MH01..euroc_V203");
    return 1;
  }

  const std::string input_path      = argv[1];
  const std::string gt_path         = argv[2];
  const std::string output_path     = argv[3];
  const std::string trajectory_type = argv[4];

  int  num_robots = 1;
  bool visualize  = false;
  for (int i = 5; i < argc; ++i) {
    if (std::string(argv[i]) == "-visualize")
      visualize = true;
    else {
      try { num_robots = std::stoi(argv[i]); }
      catch (...) { LOG_ERROR("Invalid argument: {}", argv[i]); return 1; }
    }
  }

  if (!std::ifstream(input_path)) {
    LOG_ERROR("Cannot open input file: {}", input_path);
    return 1;
  }

  // ======================= Initialize =========================================
  LOG_SECTION("Multi-Robot Stereo Visual SLAM");
  LOG_INFO("Robots: {}", num_robots);

  auto params = loadExperimentParams(getDefaultConfigPath(), trajectory_type);
  LOG_INFO("Trajectory: {} - {}", trajectory_type, params.description);
  LOG_DEBUG("qc_diag = [{}, {}, {}, {}, {}, {}]",
            params.qc_diag[0], params.qc_diag[1], params.qc_diag[2],
            params.qc_diag[3], params.qc_diag[4], params.qc_diag[5]);
  LOG_DEBUG("num_iterations = {}", params.num_iterations);

  omp_set_num_threads(omp_get_max_threads());
  LOG_DEBUG("Using {} OpenMP threads", omp_get_max_threads());

  // ======================= Load Data ==========================================
  LOG_STEP(1, 4, "Loading stereo g2o: {}", input_path);

  CameraParams              camera;
  std::vector<PoseInit>     all_poses;
  std::vector<LandmarkInit> all_landmarks;
  std::vector<StereoMeas>   all_stereo;
  std::vector<PriorMeas>    all_priors;
  parseStereoG2OFile(input_path, camera, all_poses, all_landmarks, all_stereo, all_priors);

  LOG_INFO("Loaded: {} poses, {} landmarks, {} stereo observations",
           all_poses.size(), all_landmarks.size(), all_stereo.size());

  // ======================= Split Trajectory Per Robot =========================
  LOG_STEP(2, 4, "Building {} factor graph(s)...", num_robots);

  const size_t poses_per_robot = all_poses.size() / static_cast<size_t>(num_robots);

  std::vector<std::unique_ptr<FactorGraph>> graphs(num_robots);
  std::vector<std::unique_ptr<GbpSolver>>  solvers(num_robots);
  for (int r = 0; r < num_robots; ++r) {
    graphs[r]  = std::make_unique<FactorGraph>();
    solvers[r] = std::make_unique<GbpSolver>(SolverScheduleType::SYNCHRONOUS);
  }

  for (int r = 0; r < num_robots; ++r) {
    const size_t start = static_cast<size_t>(r) * poses_per_robot;
    const size_t end   = (r == num_robots - 1) ? all_poses.size()
                                                : start + poses_per_robot;
    std::vector<PoseInit> robot_poses(all_poses.begin() + start, all_poses.begin() + end);

    LOG_INFO("Robot {}: {} poses ({}s - {}s)",
             r, robot_poses.size(), robot_poses.front().timestamp, robot_poses.back().timestamp);

    // Pose and GP-prior variables
    add_pose_variable_nodes(*graphs[r], robot_poses);
    add_gp_prior_factor_nodes(*graphs[r], params.qc_diag);

    // Collect landmarks and stereo edges that belong to this robot's time window
    std::set<size_t> robot_lm_ids;
    std::vector<StereoMeas> robot_stereo;
    for (const auto& obs : all_stereo) {
      bool in_window = false;
      for (const auto& p : robot_poses) {
        if (std::abs(p.timestamp - obs.timestamp) < 1e-9) { in_window = true; break; }
      }
      if (in_window) {
        robot_stereo.push_back(obs);
        robot_lm_ids.insert(obs.landmark_id);
      }
    }

    std::vector<LandmarkInit> robot_landmarks;
    for (const auto& lm : all_landmarks)
      if (robot_lm_ids.count(lm.landmark_id))
        robot_landmarks.push_back(lm);

    LOG_INFO("  Robot {}: {} landmarks, {} observations", r, robot_landmarks.size(), robot_stereo.size());

    add_landmark_variable_nodes(*graphs[r], robot_landmarks, r);
    add_stereo_factor_nodes(*graphs[r], robot_stereo, camera, r);

    LOG_DEBUG("  Graph {}: {} variables, {} factors",
              r, graphs[r]->variable_nodes_.size(), graphs[r]->factor_nodes_.size());
  }

  // Free raw data no longer needed
  all_poses.clear();     all_poses.shrink_to_fit();
  all_landmarks.clear(); all_landmarks.shrink_to_fit();
  all_stereo.clear();    all_stereo.shrink_to_fit();

  // Inter-robot landmark equality factors
  if (num_robots > 1) {
    std::vector<FactorGraph*> graph_ptrs(num_robots);
    for (int r = 0; r < num_robots; ++r)
      graph_ptrs[r] = graphs[r].get();
    add_landmark_equality_factor_nodes(graph_ptrs);
  }

  // ======================= Optimize with GBP ==================================
  LOG_STEP(3, 4, "Running GBP ({} iterations per robot)...", params.num_iterations);

  std::optional<rerun::RecordingStream> rec;
  if (visualize) {
    rec.emplace("stereo_visual");
    rec->spawn().exit_on_failure();
    for (int r = 0; r < num_robots; ++r) {
      rerun::Color color(static_cast<uint8_t>(255u * r / num_robots), 100u, 200u);
      visualizeFactorGraph(*rec, *graphs[r], "robot_" + std::to_string(r) + "/", 0,
                           std::nullopt, color);
    }
    LOG_DEBUG("Rerun visualization enabled");
  }

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int iter = 0; iter < params.num_iterations; ++iter) {
    for (int r = 0; r < num_robots; ++r)
      solvers[r]->performIteration(*graphs[r]);

    if (visualize && rec) {
      for (int r = 0; r < num_robots; ++r) {
        rerun::Color color(static_cast<uint8_t>(255u * r / num_robots), 100u, 200u);
        visualizeFactorGraph(*rec, *graphs[r], "robot_" + std::to_string(r) + "/",
                             iter + 1, std::nullopt, color);
      }
    }
    LOG_PROGRESS(iter + 1, params.num_iterations, "GBP iterations");
  }
  auto t1  = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  LOG_INFO("Optimization complete in {}ms", dur.count());

  // ======================= Interpolate and Save ================================
  LOG_STEP(4, 4, "GP-interpolating to GT timestamps and saving: {}", output_path);

  // Collect and sort all pose nodes across robots
  std::vector<std::shared_ptr<SE3PoseVel>> all_pose_nodes;
  for (int r = 0; r < num_robots; ++r) {
    for (const auto& var : graphs[r]->variable_nodes_) {
      if (var->id_.find("POSE_") != std::string::npos) {
        auto node = std::dynamic_pointer_cast<SE3PoseVel>(var);
        if (node)
          all_pose_nodes.push_back(node);
      }
    }
  }
  std::sort(all_pose_nodes.begin(), all_pose_nodes.end(),
            [](const auto& a, const auto& b) { return a->timestamp_ < b->timestamp_; });

  LOG_INFO("Total pose nodes collected: {}", all_pose_nodes.size());

  // Load GT timestamps to interpolate at
  const std::vector<double> gt_timestamps = loadGTTimestamps(gt_path);
  if (gt_timestamps.empty()) {
    LOG_ERROR("No GT timestamps loaded — cannot write interpolated output");
    return 1;
  }
  LOG_INFO("Interpolating at {} GT timestamps", gt_timestamps.size());

  // GP interpolation at GT timestamps
  std::vector<GPInterpDistribution> interp =
    GPInterpAtTimestamps(all_pose_nodes, gt_timestamps, params.qc_diag);

  // Write TUM output
  std::ofstream output_file(output_path);
  if (!output_file) {
    LOG_ERROR("Cannot open output file: {}", output_path);
    return 1;
  }
  output_file << std::fixed << std::setprecision(9);
  output_file << "#timestamp tx ty tz qx qy qz qw\n";
  for (size_t i = 0; i < interp.size(); ++i) {
    const Eigen::Vector3d&    t = interp[i].pose.translation();
    const Eigen::Quaterniond  q(interp[i].pose.linear());
    output_file << gt_timestamps[i] << " "
                << t.x() << " " << t.y() << " " << t.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  output_file.close();
  LOG_INFO("Wrote {} poses to {}", interp.size(), output_path);

  LOG_SECTION("Done");
  return 0;
}
