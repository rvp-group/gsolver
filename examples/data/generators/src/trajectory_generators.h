/**
 * @file trajectory_generators.h
 * @brief Synthetic trajectory generators for factor graph experiments.
 *
 * This file provides functions to generate various synthetic 3D trajectories
 * for testing and benchmarking pose graph optimization algorithms:
 * - Helix: Spiral path ascending along Z-axis
 * - Sphere: Path covering a sphere surface (Fibonacci spiral)
 *
 * Each generator produces a sequence of SE3 poses (position + orientation)
 * sampled at a configurable frequency (Hz).
 */

#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <gsolver/maths/geometry3d.h>

namespace examples {

  /**
   * @brief Supported trajectory types for synthetic data generation.
   */
  enum class TrajectoryType {
    HELIX, ///< Helical spiral trajectory
    SPHERE ///< Spherical surface trajectory
  };

  /**
   * @brief Map from string names to TrajectoryType enum.
   */
  inline const std::unordered_map<std::string, TrajectoryType> kTrajectoryTypeMap = {{"helix", TrajectoryType::HELIX},
                                                                                     {"sphere", TrajectoryType::SPHERE}};

  /**
   * @brief Generate a helical (spiral) trajectory.
   *
   * Creates a 3D helix where the camera/robot spirals upward along the Z-axis.
   * The orientation follows the tangent of the helix (yaw aligned with motion).
   *
   * Configuration parameters (from YAML):
   * - trajectory.hz: Sampling frequency
   * - trajectory.helix.path_length: Total arc length of the helix
   * - trajectory.helix.radius: Radius of the helix cylinder
   * - trajectory.helix.height: Total height of the helix
   * - trajectory.helix.num_circles: Number of complete revolutions
   * - trajectory.helix.circle_steps: Steps per revolution (at 1Hz)
   *
   * @param timestamps Output vector of timestamps (seconds)
   * @param trajectory Output vector of SE3 poses
   * @param config YAML configuration node
   */
  inline void
  generateHelixTrajectory(std::vector<double>& timestamps, std::vector<Eigen::Isometry3d>& trajectory, const YAML::Node& config) {
    // Extract trajectory parameters from config
    const double hz          = config["trajectory"]["hz"].as<double>();
    const double path_length = config["trajectory"]["helix"]["path_length"].as<double>();
    const double radius      = config["trajectory"]["helix"]["radius"].as<double>();
    const double height      = config["trajectory"]["helix"]["height"].as<double>();
    const int num_circles    = config["trajectory"]["helix"]["num_circles"].as<int>();
    const int circle_steps   = config["trajectory"]["helix"]["circle_steps"].as<int>();

    // Compute number of samples based on frequency
    const int num_samples   = static_cast<int>((num_circles * circle_steps - 1) * hz);
    const double step_size  = path_length / num_samples;
    const double circle_len = path_length / num_circles;

    double path_position = 0.0;
    for (int i = 0; i <= num_samples; ++i) {
      // Compute position on helix
      const double angle = 2.0 * M_PI * path_position / circle_len;

      Eigen::Vector3d translation;
      translation.x() = radius * std::cos(angle);
      translation.y() = radius * std::sin(angle);
      translation.z() = height * path_position / path_length;

      // Orientation: yaw follows the helix tangent
      const Eigen::Vector3d euler_angles(0.0, 0.0, angle); // roll, pitch, yaw
      const Eigen::Matrix3d rotation = gsolver::expMapSO3(euler_angles);

      // Construct SE3 pose
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear()          = rotation;
      pose.translation()     = translation;

      trajectory.push_back(pose);
      timestamps.push_back(static_cast<double>(i) / hz);

      path_position += step_size;
    }
  }

  /**
   * @brief Generate a spherical surface trajectory.
   *
   * Creates a path that covers a sphere surface using a Fibonacci spiral pattern.
   * This provides approximately uniform sampling over the sphere.
   *
   * Configuration parameters (from YAML):
   * - trajectory.hz: Sampling frequency
   * - trajectory.sphere.num_points: Number of waypoints on the sphere
   * - trajectory.sphere.radius: Radius of the sphere
   *
   * @param timestamps Output vector of timestamps (seconds)
   * @param trajectory Output vector of SE3 poses
   * @param config YAML configuration node
   */
  inline void generateSphereTrajectory(std::vector<double>& timestamps,
                                       std::vector<Eigen::Isometry3d>& trajectory,
                                       const YAML::Node& config) {
    // Extract trajectory parameters from config
    const double hz      = config["trajectory"]["hz"].as<double>();
    const int num_points = config["trajectory"]["sphere"]["num_points"].as<int>();
    const double radius  = config["trajectory"]["sphere"]["radius"].as<double>();

    // Compute number of samples based on frequency
    const int num_samples  = static_cast<int>((num_points - 1) * hz);
    const double step_size = 1.0 / hz;

    double path_position = 0.0;
    for (int i = 0; i <= num_samples; ++i) {
      // Fibonacci spiral on sphere (approximately uniform distribution)
      const double t = path_position / (num_points - 1);
      const double z = 1.9 * (t - 0.5); // Range: [-0.95, 0.95] to avoid poles

      // Compute x, y from Fibonacci spiral formula
      const double phi      = std::sqrt(num_points * M_PI) * std::asin(z);
      const double r_xy     = std::sqrt(1.0 - z * z);
      const double x        = radius * std::cos(phi) * r_xy;
      const double y        = radius * std::sin(phi) * r_xy;
      const double z_scaled = radius * z;

      // Orientation: uniform rotation along path
      const Eigen::Vector3d euler_angles = t * Eigen::Vector3d::Ones();
      const Eigen::Matrix3d rotation     = gsolver::expMapSO3(euler_angles);

      // Construct SE3 pose
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear()          = rotation;
      pose.translation()     = Eigen::Vector3d(x, y, z_scaled);

      trajectory.push_back(pose);
      timestamps.push_back(static_cast<double>(i) / hz);

      path_position += step_size;
    }
  }

  /**
   * @brief Generate a trajectory of the specified type.
   *
   * Dispatcher function that calls the appropriate trajectory generator
   * based on the trajectory type.
   *
   * @param type Trajectory type to generate
   * @param timestamps Output vector of timestamps (seconds)
   * @param trajectory Output vector of SE3 poses
   * @param config YAML configuration node
   * @return true if trajectory was generated successfully, false otherwise
   */
  inline bool generateTrajectory(TrajectoryType type,
                                 std::vector<double>& timestamps,
                                 std::vector<Eigen::Isometry3d>& trajectory,
                                 const YAML::Node& config) {
    switch (type) {
      case TrajectoryType::HELIX:
        generateHelixTrajectory(timestamps, trajectory, config);
        return true;
      case TrajectoryType::SPHERE:
        generateSphereTrajectory(timestamps, trajectory, config);
        return true;
      default:
        return false;
    }
  }

} // namespace examples
