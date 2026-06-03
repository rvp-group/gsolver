/**
 * @file rerun_utils.h
 * @brief Rerun visualization utilities for pose graph examples.
 */

#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/types/variables/se3_pose.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/gp_interpolation.h>

namespace examples {

  using namespace gsolver;

  /**
   * @brief Visualization parameters for GP interpolation.
   */
  struct VisualizationParams {
    Eigen::Vector<double, 6> qc_diag; ///< GP prior covariance diagonal
    int hertz = 100;                  ///< Interpolation frequency (default 100Hz)
  };

  /**
   * @brief Visualize a factor graph in Rerun.
   *
   * Displays:
   * - Trajectory as a continuous line strip (GP-interpolated if params provided)
   * - Pose waypoints as black dots
   * - Landmarks as red dots (if present)
   *
   * @param rec Rerun recording stream
   * @param factor_graph Factor graph to visualize
   * @param entity_name Entity name prefix for Rerun logging
   * @param iteration Current iteration number (used for time sequence)
   * @param interp_params Optional interpolation params (if nullopt, uses direct waypoint connection)
   * @param trajectory_color Color for trajectory line strip
   */
  inline void visualizeFactorGraph(const rerun::RecordingStream& rec,
                                   FactorGraph& factor_graph,
                                   const std::string& entity_name                          = "",
                                   const int iteration                                     = 0,
                                   const std::optional<VisualizationParams>& interp_params = std::nullopt,
                                   const rerun::Color& trajectory_color                    = rerun::Color(0, 114, 189)) {
    // Set time sequence for proper timeline in Rerun viewer
    rec.set_time_sequence("iteration", iteration);

    // Collect pose waypoints and landmarks from factor graph
    std::vector<rerun::Vec3D> waypoints;
    std::vector<rerun::Position3D> waypoint_positions;
    std::vector<rerun::Position3D> landmarks;

    for (auto& variable_node : factor_graph.variable_nodes_) {
      if (variable_node->id_.find("POSE_") != std::string::npos || variable_node->id_.find("pose_") != std::string::npos) {
        const std::shared_ptr<SE3PoseVel> variable_n = std::dynamic_pointer_cast<SE3PoseVel>(variable_node);
        if (!variable_n)
          continue;

        const Eigen::Vector3d translation = variable_n->mu_.pose.translation();
        waypoints.push_back(rerun::Vec3D(translation(0), translation(1), translation(2)));
        waypoint_positions.push_back(rerun::Position3D(translation(0), translation(1), translation(2)));
      } else if (variable_node->id_.find("LANDMARK_") != std::string::npos) {
        const std::shared_ptr<SE3Pose> variable_n = std::dynamic_pointer_cast<SE3Pose>(variable_node);
        if (!variable_n)
          continue;

        const Eigen::Vector3d translation = variable_n->mu_.translation();
        landmarks.push_back(rerun::Position3D(translation(0), translation(1), translation(2)));
      }
    }

    // Compute trajectory line strip (interpolated or direct)
    std::vector<rerun::Vec3D> trajectory_strip;
    if (interp_params.has_value() && factor_graph.getPoseVariableNodes().size() >= 2) {
      // GP-interpolated trajectory
      std::vector<GPInterpDistribution> interp_results = GPInterp(factor_graph, interp_params->qc_diag, interp_params->hertz);
      trajectory_strip.reserve(interp_results.size());
      for (const auto& interp : interp_results) {
        const Eigen::Vector3d t = interp.pose.translation();
        trajectory_strip.push_back(rerun::Vec3D(t(0), t(1), t(2)));
      }
    } else {
      // Direct waypoint connection
      trajectory_strip = waypoints;
    }

    // Log trajectory as continuous line strip
    if (!trajectory_strip.empty()) {
      std::vector<rerun::components::LineStrip3D> strips = {rerun::components::LineStrip3D(trajectory_strip)};
      rec.log(entity_name + "trajectory", rerun::LineStrips3D(strips).with_colors(trajectory_color));
    }

    // Log pose waypoints as black dots
    if (!waypoint_positions.empty()) {
      rec.log(entity_name + "poses", rerun::Points3D(waypoint_positions).with_colors(rerun::Color(0, 0, 0)).with_radii({0.02f}));
    }

    // Log landmarks as red dots
    if (!landmarks.empty()) {
      rec.log(entity_name + "landmarks", rerun::Points3D(landmarks).with_colors(rerun::Color(255, 0, 0)).with_radii({0.01f}));
    }
  }

} // namespace examples