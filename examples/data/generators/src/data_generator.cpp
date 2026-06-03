/**
 * @file data_generator.cpp
 * @brief Synthetic dataset generator for pose graph optimization experiments.
 *
 * This tool generates synthetic factor graph datasets in G2O format for
 * benchmarking pose graph optimization algorithms. It supports:
 * - Multiple trajectory types (helix, sphere)
 * - Configurable noise levels for initial guesses and measurements
 * - Prior factors (absolute pose measurements)
 * - Pose-pose factors (relative odometry measurements)
 * - Ground truth trajectory output in TUM format
 *
 * Usage: ./data_generator <config.yaml> <helix|sphere> <output_prefix>
 *
 * The config YAML file specifies trajectory parameters, noise levels,
 * and which factor types to include. See generate_configs.py for examples.
 */

#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "trajectory_generators.h"
#include <gsolver/maths/geometry3d.h>

// Logging (simple version for standalone tool)
#define LOG_INFO(msg) std::cout << "[INFO ] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl

using namespace gsolver;
using namespace examples;

using Vector6d = Eigen::Matrix<double, 6, 1>;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Convert upper-triangular part of a matrix to a space-separated string.
 *
 * G2O uses the upper-triangular part of the information matrix in row-major order.
 *
 * @param mat Input matrix (typically 6x6 covariance or information matrix)
 * @return String representation for G2O format
 */
std::string matrixToUpperTriangularString(const Eigen::MatrixXd& mat) {
  std::ostringstream ss;
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = i; j < mat.cols(); ++j) {
      ss << mat(i, j) << " ";
    }
  }
  return ss.str();
}

/**
 * @brief Add Gaussian noise to an SE3 pose.
 *
 * @param pose Input pose to perturb
 * @param trans_sigma Standard deviation for translation noise (meters)
 * @param rot_sigma Standard deviation for rotation noise (radians)
 * @param gen Random number generator
 * @return Perturbed pose
 */
Eigen::Isometry3d addNoiseToPose(const Eigen::Isometry3d& pose, double trans_sigma, double rot_sigma, std::mt19937& gen) {
  std::normal_distribution<double> trans_noise(0.0, trans_sigma);
  std::normal_distribution<double> rot_noise(0.0, rot_sigma);

  Vector6d noise;
  noise << trans_noise(gen), trans_noise(gen), trans_noise(gen), rot_noise(gen), rot_noise(gen), rot_noise(gen);

  return pose * expMapSE3(noise);
}

// ============================================================================
// G2O Writers
// ============================================================================

/**
 * @brief Write SE3 vertices to a G2O file.
 *
 * Writes VERTEX_SE3 entries with initial pose estimates and covariance.
 *
 * @param timestamps Vector of timestamps
 * @param trajectory Vector of ground truth poses
 * @param config YAML configuration
 * @param gen Random number generator
 * @param filename Output filename
 */
void writeG2OVertices(const std::vector<double>& timestamps,
                      const std::vector<Eigen::Isometry3d>& trajectory,
                      const YAML::Node& config,
                      std::mt19937& gen,
                      const std::string& filename) {
  // Get noise parameters
  const bool add_noise     = config["variable_node"]["std_dev_data"]["add_noise"].as<bool>();
  const double trans_sigma = config["variable_node"]["std_dev_data"]["translation"].as<double>();
  const double rot_sigma   = config["variable_node"]["std_dev_data"]["rotation"].as<double>();

  // Compute covariance matrix
  Eigen::Matrix<double, 6, 6> sigma = Eigen::Matrix<double, 6, 6>::Identity();
  sigma.block<3, 3>(0, 0) *= trans_sigma * trans_sigma;
  sigma.block<3, 3>(3, 3) *= rot_sigma * rot_sigma;

  std::ofstream file(filename);
  if (!file.is_open()) {
    LOG_ERROR("Cannot open file: " << filename);
    return;
  }

  for (size_t i = 0; i < trajectory.size(); ++i) {
    Eigen::Isometry3d pose = trajectory[i];

    // Add noise if configured
    if (add_noise && (trans_sigma > 0 || rot_sigma > 0)) {
      pose = addNoiseToPose(pose, trans_sigma, rot_sigma, gen);
    }

    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d& t = pose.translation();

    file << "VERTEX_SE3 " << timestamps[i] << " " << t.x() << " " << t.y() << " " << t.z() << " " << quat.x() << " " << quat.y()
         << " " << quat.z() << " " << quat.w() << " " << matrixToUpperTriangularString(sigma) << "\n";
  }

  file.close();
  LOG_INFO("Generated " << trajectory.size() << " vertices in " << filename);
}

/**
 * @brief Write ground truth trajectory to TUM format file.
 *
 * TUM format: timestamp tx ty tz qx qy qz qw
 *
 * @param timestamps Vector of timestamps
 * @param trajectory Vector of ground truth poses
 * @param filename Output filename
 */
void writeGroundTruthTUM(const std::vector<double>& timestamps,
                         const std::vector<Eigen::Isometry3d>& trajectory,
                         const std::string& filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    LOG_ERROR("Cannot open file: " << filename);
    return;
  }

  file << "#timestamp tx ty tz qx qy qz qw\n";

  for (size_t i = 0; i < trajectory.size(); ++i) {
    const Eigen::Quaterniond quat(trajectory[i].linear());
    const Eigen::Vector3d& t = trajectory[i].translation();

    file << timestamps[i] << " " << t.x() << " " << t.y() << " " << t.z() << " " << quat.x() << " " << quat.y() << " " << quat.z()
         << " " << quat.w() << "\n";
  }

  file.close();
  LOG_INFO("Generated ground truth with " << trajectory.size() << " poses in " << filename);
}

/**
 * @brief Append prior factor edges to a G2O file.
 *
 * Prior factors provide absolute pose measurements (e.g., from GPS or markers).
 *
 * @param timestamps Vector of timestamps
 * @param trajectory Vector of ground truth poses
 * @param config YAML configuration
 * @param gen Random number generator
 * @param filename Output filename (append mode)
 */
void appendG2OPriorEdges(const std::vector<double>& timestamps,
                         const std::vector<Eigen::Isometry3d>& trajectory,
                         const YAML::Node& config,
                         std::mt19937& gen,
                         const std::string& filename) {
  // Get noise parameters
  const bool add_noise     = config["factor_node"]["prior"]["std_dev_data"]["add_noise"].as<bool>();
  const double trans_sigma = config["factor_node"]["prior"]["std_dev_data"]["translation"].as<double>();
  const double rot_sigma   = config["factor_node"]["prior"]["std_dev_data"]["rotation"].as<double>();

  // Compute covariance matrix
  Eigen::Matrix<double, 6, 6> sigma = Eigen::Matrix<double, 6, 6>::Identity();
  sigma.block<3, 3>(0, 0) *= trans_sigma * trans_sigma;
  sigma.block<3, 3>(3, 3) *= rot_sigma * rot_sigma;

  std::ofstream file(filename, std::ios::app);
  if (!file.is_open()) {
    LOG_ERROR("Cannot open file: " << filename);
    return;
  }

  for (size_t i = 0; i < trajectory.size(); ++i) {
    Eigen::Isometry3d measurement = trajectory[i];

    // Add noise if configured
    if (add_noise && (trans_sigma > 0 || rot_sigma > 0)) {
      measurement = addNoiseToPose(measurement, trans_sigma, rot_sigma, gen);
    }

    const Eigen::Quaterniond quat(measurement.linear());
    const Eigen::Vector3d& t = measurement.translation();

    file << "EDGE_PRIOR_SE3 " << timestamps[i] << " " << t.x() << " " << t.y() << " " << t.z() << " " << quat.x() << " "
         << quat.y() << " " << quat.z() << " " << quat.w() << " " << matrixToUpperTriangularString(sigma) << "\n";
  }

  file.close();
  LOG_INFO("Generated " << trajectory.size() << " prior factors");
}

/**
 * @brief Append pose-pose (odometry) factor edges to a G2O file.
 *
 * Generates:
 * - Consecutive edges between all adjacent poses
 * - Random loop closure edges based on configuration
 *
 * @param timestamps Vector of timestamps
 * @param trajectory Vector of ground truth poses
 * @param config YAML configuration
 * @param gen Random number generator
 * @param filename Output filename (append mode)
 */
void appendG2OPosePoseEdges(const std::vector<double>& timestamps,
                            const std::vector<Eigen::Isometry3d>& trajectory,
                            const YAML::Node& config,
                            std::mt19937& gen,
                            const std::string& filename) {
  // Get noise parameters
  const bool add_noise     = config["factor_node"]["pose_pose"]["std_dev_data"]["add_noise"].as<bool>();
  const double trans_sigma = config["factor_node"]["pose_pose"]["std_dev_data"]["translation"].as<double>();
  const double rot_sigma   = config["factor_node"]["pose_pose"]["std_dev_data"]["rotation"].as<double>();

  // Compute covariance matrix
  Eigen::Matrix<double, 6, 6> sigma = Eigen::Matrix<double, 6, 6>::Identity();
  sigma.block<3, 3>(0, 0) *= trans_sigma * trans_sigma;
  sigma.block<3, 3>(3, 3) *= rot_sigma * rot_sigma;

  // Get loop closure percentage
  const double loop_closure_pct  = config["factor_node"]["pose_pose"]["percentage_of_interconnections"].as<double>();
  const size_t num_loop_closures = static_cast<size_t>(loop_closure_pct * trajectory.size() / 100.0);

  std::ofstream file(filename, std::ios::app);
  if (!file.is_open()) {
    LOG_ERROR("Cannot open file: " << filename);
    return;
  }

  size_t edge_count = 0;

  // Write consecutive pose-pose edges (odometry)
  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    const size_t from_id = i;
    const size_t to_id   = i + 1;

    // Compute relative transformation
    Eigen::Isometry3d relative = trajectory[from_id].inverse() * trajectory[to_id];

    // Add noise if configured
    if (add_noise && (trans_sigma > 0 || rot_sigma > 0)) {
      relative = addNoiseToPose(relative, trans_sigma, rot_sigma, gen);
    }

    const Eigen::Quaterniond quat(relative.linear());
    const Eigen::Vector3d& t = relative.translation();

    file << "EDGE_POSE_POSE_SE3 " << timestamps[from_id] << " " << timestamps[to_id] << " " << t.x() << " " << t.y() << " "
         << t.z() << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
         << matrixToUpperTriangularString(sigma) << "\n";

    edge_count++;
  }

  // Generate random loop closure edges
  std::uniform_int_distribution<size_t> dist(0, trajectory.size() - 1);
  for (size_t i = 0; i < num_loop_closures; ++i) {
    size_t from_id, to_id;

    // Find valid non-adjacent pair
    do {
      from_id = dist(gen);
      to_id   = dist(gen);
    } while (from_id == to_id || std::abs(static_cast<int>(from_id) - static_cast<int>(to_id)) == 1);

    // Compute relative transformation
    Eigen::Isometry3d relative = trajectory[from_id].inverse() * trajectory[to_id];

    // Add noise if configured
    if (add_noise && (trans_sigma > 0 || rot_sigma > 0)) {
      relative = addNoiseToPose(relative, trans_sigma, rot_sigma, gen);
    }

    const Eigen::Quaterniond quat(relative.linear());
    const Eigen::Vector3d& t = relative.translation();

    file << "EDGE_POSE_POSE_SE3 " << timestamps[from_id] << " " << timestamps[to_id] << " " << t.x() << " " << t.y() << " "
         << t.z() << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
         << matrixToUpperTriangularString(sigma) << "\n";

    edge_count++;
  }

  file.close();
  LOG_INFO("Generated " << edge_count << " pose-pose factors (" << num_loop_closures << " loop closures)");
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
  // Parse command line arguments
  if (argc < 4) {
    std::cout << "Usage: " << argv[0] << " <config.yaml> <helix|sphere> <output_prefix>\n";
    std::cout << "\n";
    std::cout << "Arguments:\n";
    std::cout << "  config.yaml    - YAML configuration file\n";
    std::cout << "  trajectory     - Trajectory type: helix, sphere\n";
    std::cout << "  output_prefix  - Output file prefix (without extension)\n";
    std::cout << "\n";
    std::cout << "Output:\n";
    std::cout << "  If gt=true:  <output_prefix>.tum (ground truth trajectory)\n";
    std::cout << "  If gt=false: <output_prefix>.g2o (factor graph)\n";
    return 1;
  }

  const std::string config_path    = argv[1];
  const std::string trajectory_str = argv[2];
  const std::string output_prefix  = argv[3];

  LOG_INFO("Data Generator: " << trajectory_str << " trajectory");
  LOG_DEBUG("Config: " << config_path);
  LOG_DEBUG("Output: " << output_prefix);

  // Load configuration
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (const std::exception& e) {
    LOG_ERROR("Failed to load config: " << e.what());
    return 1;
  }

  // Check trajectory type
  auto it = kTrajectoryTypeMap.find(trajectory_str);
  if (it == kTrajectoryTypeMap.end()) {
    LOG_ERROR("Unknown trajectory type: " << trajectory_str);
    LOG_INFO("Supported types: helix, sphere");
    return 1;
  }
  const TrajectoryType trajectory_type = it->second;

  // Initialize random generator
  const int random_seed = config["random_seed"].as<int>();
  std::mt19937 gen(random_seed);
  LOG_DEBUG("Random seed: " << random_seed);

  // Generate trajectory
  std::vector<double> timestamps;
  std::vector<Eigen::Isometry3d> trajectory;
  if (!generateTrajectory(trajectory_type, timestamps, trajectory, config)) {
    LOG_ERROR("Failed to generate trajectory");
    return 1;
  }
  LOG_INFO("Generated " << trajectory.size() << " poses");

  // Output based on mode
  const bool is_ground_truth = config["gt"].as<bool>();

  if (is_ground_truth) {
    // Write ground truth in TUM format
    const std::string filename = output_prefix + ".tum";
    writeGroundTruthTUM(timestamps, trajectory, filename);
  } else {
    // Write factor graph in G2O format
    const std::string filename = output_prefix + ".g2o";
    writeG2OVertices(timestamps, trajectory, config, gen, filename);

    // Add prior factors if configured
    if (config["factor_node"]["prior"]["add_factors"].as<bool>()) {
      appendG2OPriorEdges(timestamps, trajectory, config, gen, filename);
    }

    // Add pose-pose factors if configured
    if (config["factor_node"]["pose_pose"]["add_factors"].as<bool>()) {
      appendG2OPosePoseEdges(timestamps, trajectory, config, gen, filename);
    }
  }

  LOG_INFO("Done: " << output_prefix << (is_ground_truth ? ".tum" : ".g2o"));
  return 0;
}
