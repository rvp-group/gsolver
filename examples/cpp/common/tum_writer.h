/**
 * @file tum_writer.h
 * @brief Writer for TUM trajectory format files.
 */

#pragma once

#include "data_types.h"
#include "logger.h"

#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/graph/variable_node_base.h>

#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace gsolver;

namespace examples {

  /**
   * @brief Write poses to a TUM format file.
   *
   * TUM format: timestamp tx ty tz qx qy qz qw
   *
   * @param filename Output file path
   * @param poses Vector of initial poses to write
   */
  inline void writeTUM(const std::string& filename, const std::vector<PoseInit>& poses) {
    std::ofstream file(filename);
    if (!file.is_open()) {
      LOG_ERROR("Could not open file for writing: {}", filename);
      return;
    }

    file << std::fixed << std::setprecision(9);

    for (const auto& pose : poses) {
      const Eigen::Vector3d& t = pose.pose.translation();
      const Eigen::Quaterniond q(pose.pose.linear());

      file << pose.timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z()
           << " " << q.w() << "\n";
    }

    LOG_DEBUG("Written {} poses to {}", poses.size(), filename);
  }

  /**
   * @brief Write poses with timestamps from a separate vector.
   *
   * @param filename Output file path
   * @param timestamps Vector of timestamps
   * @param poses Vector of Isometry3d poses
   */
  inline void
  writeTUM(const std::string& filename, const std::vector<double>& timestamps, const std::vector<Eigen::Isometry3d>& poses) {
    if (timestamps.size() != poses.size()) {
      LOG_ERROR("timestamps and poses vectors must have the same size");
      return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
      LOG_ERROR("Could not open file for writing: {}", filename);
      return;
    }

    file << std::fixed << std::setprecision(9);

    for (size_t i = 0; i < poses.size(); ++i) {
      const Eigen::Vector3d& t = poses[i].translation();
      const Eigen::Quaterniond q(poses[i].linear());

      file << timestamps[i] << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
           << q.w() << "\n";
    }

    LOG_DEBUG("Written {} poses to {}", poses.size(), filename);
  }

  inline void writeTUM(FactorGraph& factor_graph, std::ofstream& output_file) {
    const std::vector<std::shared_ptr<VariableNodeBase>> pose_variable_nodes = factor_graph.getPoseVariableNodes();
    output_file << "#timestamp tx ty tz qx qy qz qw"
                << "\n";
    for (auto& variable_node : pose_variable_nodes) {
      if (variable_node->id_.find("POSE_") != std::string::npos) {
        const std::shared_ptr<SE3PoseVel> variable_n = std::dynamic_pointer_cast<SE3PoseVel>(variable_node);
        const SE3PoseVelValue mu                     = SE3PoseVelValue(variable_n->mu_);
        const Eigen::Vector3d translation            = mu.pose.translation();
        const Eigen::Quaterniond rotation(mu.pose.linear());

        std::string timestamp = variable_n->id_.substr(5);
        output_file << timestamp << " " << translation(0) << " " << translation(1) << " " << translation(2) << " " << rotation.x()
                    << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << "\n";
      }
    }

    LOG_DEBUG("Written {} poses to output file", pose_variable_nodes.size());
  }

} // namespace examples
