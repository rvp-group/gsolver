/**
 * @file factor_graph_builder.h
 * @brief Utilities for constructing factor graphs from g2o data files.
 *
 * This header provides helper functions to build factor graphs for SLAM/PGO:
 *
 * Variable Nodes:
 *   - add_pose_variable_node()     : Add a single SE3 pose+velocity variable
 *   - add_pose_variable_nodes()    : Add all poses from parsed g2o vertices
 *
 * Factor Nodes:
 *   - add_odometry_factor_nodes()  : Add relative pose constraints (odometry)
 *   - add_gp_prior_factor_node()   : Add a single GP motion prior between two poses
 *   - add_gp_prior_factor_nodes()  : Add GP motion priors between all consecutive poses
 *   - add_prior_factor_nodes()     : Add absolute pose priors
 *   - add_landmark_factor()        : Add pose-to-landmark measurement (AprilTag)
 *
 * @note All functions use timestamp-based variable IDs: "POSE_<timestamp>"
 * @note Landmarks use IDs: "LANDMARK_<id>"
 */

#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "data_types.h"

// gsolver library
#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/types/factors/se3_posevel_gp.h>
#include <gsolver/graph/types/factors/se3_posevel_pose.h>
#include <gsolver/graph/types/factors/se3_posevel_posevel.h>
#include <gsolver/graph/types/factors/se3_posevel_prior.h>
#include <gsolver/graph/types/variables/se3_pose.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/geometry3d.h>

using namespace gsolver;

namespace examples {

  // ============================================================================
  // Helper Functions
  // ============================================================================

  /**
   * @brief Convert timestamp to string with fixed precision for ID generation.
   * @param timestamp The timestamp value
   * @return String representation with 9 decimal places
   */
  inline std::string timestampToString(double timestamp) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << timestamp;
    return oss.str();
  }

  /**
   * @brief Generate pose variable ID from timestamp.
   * @param timestamp The pose timestamp
   * @return Variable ID string "POSE_<timestamp>"
   */
  inline std::string poseId(double timestamp) {
    return "POSE_" + timestampToString(timestamp);
  }

  /**
   * @brief Generate landmark variable ID from landmark index.
   * @param landmark_id The landmark index
   * @return Variable ID string "LANDMARK_<id>"
   */
  inline std::string landmarkId(int landmark_id) {
    return "LANDMARK_" + std::to_string(landmark_id);
  }

  // ============================================================================
  // Variable Nodes
  // ============================================================================

  /**
   * @brief Add a single pose variable node to the factor graph.
   *
   * Creates an SE3PoseVel variable with pose and velocity state.
   *
   * @param factor_graph The factor graph to add the variable to
   * @param pose         The initial pose data (timestamp, SE3 transform, covariance)
   * @param velocity     Initial velocity estimate (6D: linear + angular)
   * @param sigma        Initial covariance (12x12 for pose+velocity)
   */
  inline void add_pose_variable_node(FactorGraph& factor_graph,
                                     const PoseInit& pose,
                                     const Eigen::Vector<double, 6>& velocity,
                                     const Eigen::Matrix<double, 12, 12>& sigma) {
    SE3PoseVelValue mu;
    mu.pose = pose.pose;
    mu.vel  = velocity;

    auto variable = std::make_shared<SE3PoseVel>(poseId(pose.timestamp), mu, sigma, pose.timestamp);
    factor_graph.addVariableNode(variable);
  }

  /**
   * @brief Add all pose variable nodes from parsed g2o initial poses.
   *
   * Estimates initial velocities from finite differences between consecutive poses.
   * Uses a default covariance of 3*I for both pose and velocity components.
   *
   * @param factor_graph The factor graph to add variables to
   * @param poses        Vector of initial poses from g2o file
   */
  inline void add_pose_variable_nodes(FactorGraph& factor_graph, const std::vector<PoseInit>& poses) {
    // Estimate velocities from pose differences
    std::vector<Eigen::Vector<double, 6>> velocities;
    velocities.reserve(poses.size());

    for (size_t i = 0; i < poses.size() - 1; ++i) {
      double dt                  = poses[i + 1].timestamp - poses[i].timestamp;
      Eigen::Isometry3d delta    = poses[i].pose.inverse() * poses[i + 1].pose;
      Eigen::Vector<double, 6> v = logMapSE3(delta) / dt;
      velocities.push_back(v);
    }
    velocities.push_back(velocities.back()); // Last pose uses same velocity

    // Default covariance
    Eigen::Matrix<double, 12, 12> sigma = Eigen::Matrix<double, 12, 12>::Identity() * 3.0;

    // Add all variables
    for (size_t i = 0; i < poses.size(); ++i) {
      add_pose_variable_node(factor_graph, poses[i], velocities[i], sigma);
    }
  }

  // ============================================================================
  // Factor Nodes - Odometry (Relative Pose Constraints)
  // ============================================================================

  /**
   * @brief Add odometry factors from parsed g2o edges.
   *
   * Creates SE3PoseVelPoseVel factors representing relative pose measurements
   * between consecutive poses (wheel odometry, visual odometry, etc.).
   *
   * @param factor_graph The factor graph to add factors to
   * @param odometry     Vector of odometry measurements from g2o file
   */
  inline void add_odometry_factor_nodes(FactorGraph& factor_graph, const std::vector<OdometryMeas>& odometry) {
    for (const auto& meas : odometry) {
      // Add small regularization to covariance
      Eigen::Matrix<double, 6, 6> cov = meas.covariance + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();

      // Factor ID: "ODOM_<from_timestamp>_<to_timestamp>"
      std::string factor_id = "ODOM_" + timestampToString(meas.timestamp_from) + "_" + timestampToString(meas.timestamp_to);

      auto factor = std::make_shared<SE3PoseVelPoseVel>(factor_id, meas.measurement, cov);
      factor_graph.addFactorNode(factor, {poseId(meas.timestamp_from), poseId(meas.timestamp_to)});
    }
  }

  // ============================================================================
  // Factor Nodes - GP Motion Prior
  // ============================================================================

  /**
   * @brief Add a single GP motion prior factor between two poses.
   *
   * The GP prior enforces smooth constant-velocity motion using the covariance:
   *   Q_d = | (dt^3/3)*Qc   (dt^2/2)*Qc |
   *         | (dt^2/2)*Qc      dt*Qc    |
   *
   * @param factor_graph   The factor graph to add the factor to
   * @param from_timestamp Timestamp of the first pose
   * @param to_timestamp   Timestamp of the second pose
   * @param Qc_diag        Diagonal of the continuous-time power spectral density (6D)
   */
  inline void add_gp_prior_factor_node(FactorGraph& factor_graph,
                                       double from_timestamp,
                                       double to_timestamp,
                                       const Eigen::Vector<double, 6>& Qc_diag) {
    double dt = to_timestamp - from_timestamp;
    if (dt <= 0) {
      std::cerr << "[GP Prior] Error: Non-positive time difference dt=" << dt << std::endl;
      return;
    }

    // Build Qc with minimum values for numerical stability
    Eigen::Matrix<double, 6, 6> Qc = Qc_diag.cwiseMax(1e-4).asDiagonal();

    // Compute discrete-time covariance Q_d
    Eigen::Matrix<double, 12, 12> Qd = Eigen::Matrix<double, 12, 12>::Zero();
    Qd.block<6, 6>(0, 0)             = std::pow(dt, 3) / 3.0 * Qc;
    Qd.block<6, 6>(0, 6)             = std::pow(dt, 2) / 2.0 * Qc;
    Qd.block<6, 6>(6, 0)             = std::pow(dt, 2) / 2.0 * Qc;
    Qd.block<6, 6>(6, 6)             = dt * Qc;

    // Factor ID: "GP_<from_timestamp>_<to_timestamp>"
    std::string factor_id = "GP_" + timestampToString(from_timestamp) + "_" + timestampToString(to_timestamp);

    auto factor = std::make_shared<SE3PoseVelGP>(factor_id, dt, Qd);
    factor_graph.addFactorNode(factor, {poseId(from_timestamp), poseId(to_timestamp)});
  }

  /**
   * @brief Add GP motion prior factors between all consecutive poses.
   *
   * Iterates through all pose variables in temporal order and adds GP priors.
   *
   * @param factor_graph The factor graph (must already contain pose variables)
   * @param Qc_diag      Diagonal of the continuous-time power spectral density (6D)
   */
  inline void add_gp_prior_factor_nodes(FactorGraph& factor_graph, const Eigen::Vector<double, 6>& Qc_diag) {
    auto pose_nodes = factor_graph.getPoseVariableNodes();

    for (size_t i = 0; i < pose_nodes.size() - 1; ++i) {
      // Extract timestamps from variable IDs (format: "POSE_<timestamp>")
      double from_ts = std::stod(pose_nodes[i]->id_.substr(5));
      double to_ts   = std::stod(pose_nodes[i + 1]->id_.substr(5));

      add_gp_prior_factor_node(factor_graph, from_ts, to_ts, Qc_diag);
    }
  }

  // ============================================================================
  // Factor Nodes - Prior (Absolute Pose Constraints)
  // ============================================================================

  /**
   * @brief Add absolute pose prior factors from parsed g2o priors.
   *
   * Prior factors anchor specific poses to known absolute values
   * (e.g., GPS measurements, known starting position).
   *
   * @param factor_graph The factor graph to add factors to
   * @param priors       Vector of prior measurements from g2o file
   */
  inline void add_prior_factor_nodes(FactorGraph& factor_graph, const std::vector<PriorMeas>& priors) {
    for (const auto& prior : priors) {
      // Add small regularization to covariance
      Eigen::Matrix<double, 6, 6> cov = prior.covariance + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();

      // Factor ID: "PRIOR_<timestamp>"
      std::string factor_id = "PRIOR_" + timestampToString(prior.timestamp);

      auto factor = std::make_shared<SE3PoseVelPrior>(factor_id, prior.pose, cov);
      factor_graph.addFactorNode(factor, {poseId(prior.timestamp)});
    }
  }

  // ============================================================================
  // Factor Nodes - Landmark (Pose-to-Landmark Measurements)
  // ============================================================================

  /**
   * @brief Add a landmark measurement factor (e.g., AprilTag detection).
   *
   * Creates a factor between a pose and a landmark. If the landmark doesn't
   * exist yet, it is initialized from the current pose and measurement.
   *
   * @param factor_graph The factor graph to add the factor to
   * @param landmark     Landmark measurement data (pose timestamp, landmark ID, relative transform)
   * @param sigma        Measurement covariance (6x6), defaults to 3*I
   */
  inline void add_landmark_factor(FactorGraph& factor_graph,
                                  const LandmarkMeas& landmark,
                                  const Eigen::Matrix<double, 6, 6>& sigma = Eigen::Matrix<double, 6, 6>::Identity() * 3.0) {
    std::string pose_var_id     = poseId(landmark.timestamp);
    std::string landmark_var_id = landmarkId(landmark.landmark_id);

    // Initialize landmark if it doesn't exist
    if (!factor_graph.getVariableNode(landmark_var_id)) {
      auto pose_node       = std::dynamic_pointer_cast<SE3PoseVel>(factor_graph.getVariableNode(pose_var_id));
      Eigen::Isometry3d lm = pose_node->mu_.pose * landmark.measurement;
      auto landmark_var    = std::make_shared<SE3Pose>(landmark_var_id, lm, sigma);
      factor_graph.addVariableNode(landmark_var);
    }

    // Factor ID: "<timestamp>_<landmark_id>"
    std::string factor_id = timestampToString(landmark.timestamp) + "_" + std::to_string(landmark.landmark_id);

    auto factor = std::make_shared<SE3PoseVelPose>(factor_id, landmark.measurement, sigma);
    factor_graph.addFactorNode(factor, {pose_var_id, landmark_var_id});
  }

  /**
   * @brief Add all landmark factors from parsed g2o landmark measurements.
   *
   * Batch version of add_landmark_factor for convenience.
   *
   * @param factor_graph The factor graph to add factors to
   * @param landmarks    Vector of landmark measurements from g2o file
   * @param sigma        Measurement covariance (6x6), defaults to 3*I
   */
  inline void add_landmark_factors(FactorGraph& factor_graph,
                                   const std::vector<LandmarkMeas>& landmarks,
                                   const Eigen::Matrix<double, 6, 6>& sigma = Eigen::Matrix<double, 6, 6>::Identity() * 3.0) {
    for (const auto& landmark : landmarks) {
      add_landmark_factor(factor_graph, landmark, sigma);
    }
  }

  // ============================================================================
  // Utility Functions
  // ============================================================================

  /**
   * @brief Fix the first pose to remove gauge freedom.
   *
   * Adds a strong prior on the first pose to anchor the trajectory.
   * Essential for bundle adjustment to avoid drift in the absolute reference frame.
   *
   * @param factor_graph The factor graph (must already contain pose variables)
   * @param pose         The pose to anchor to (defaults to identity)
   * @param sigma        Prior covariance - smaller = stronger constraint (defaults to 1e-6*I)
   */
  inline void
  fix_first_pose(FactorGraph& factor_graph, const Eigen::Isometry3d& pose = Eigen::Isometry3d::Identity(), double sigma = 1e-6) {
    auto first_node                 = std::dynamic_pointer_cast<SE3PoseVel>(factor_graph.getPoseVariableNodes().at(0));
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * sigma;

    std::string factor_id = "PRIOR_ANCHOR_" + timestampToString(first_node->timestamp_);
    auto factor           = std::make_shared<SE3PoseVelPrior>(factor_id, pose, cov);
    factor_graph.addFactorNode(factor, {first_node->id_});
  }

} // namespace examples
