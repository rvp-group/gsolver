/**
 * @file data_types.h
 * @brief Data structures for parsing g2o files.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace examples {

  /**
   * @brief Initial pose guess parsed from g2o vertex.
   */
  struct PoseInit {
    double timestamp;
    Eigen::Isometry3d pose;
    Eigen::Matrix<double, 6, 6> covariance;

    PoseInit() = default;

    explicit PoseInit(const std::vector<std::string>& tokens) {
      timestamp = std::stod(tokens[1]);
      Eigen::Vector3d position(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));
      Eigen::Quaterniond orientation(std::stod(tokens[8]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]));

      pose               = Eigen::Isometry3d::Identity();
      pose.translation() = position;
      pose.linear()      = orientation.toRotationMatrix();

      covariance = Eigen::Matrix<double, 6, 6>::Zero();
      size_t idx = 9;
      for (int row = 0; row < 6; ++row) {
        for (int col = row; col < 6; ++col) {
          covariance(row, col) = std::stod(tokens[idx++]);
          covariance(col, row) = covariance(row, col);
        }
      }
    }
  };

  /**
   * @brief Odometry measurement (relative pose between two timestamps).
   */
  struct OdometryMeas {
    double timestamp_from;
    double timestamp_to;
    Eigen::Isometry3d measurement;
    Eigen::Matrix<double, 6, 6> covariance;

    explicit OdometryMeas(const std::vector<std::string>& tokens) {
      timestamp_from = std::stod(tokens[1]);
      timestamp_to   = std::stod(tokens[2]);
      Eigen::Vector3d translation(std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]));
      Eigen::Quaterniond rotation(std::stod(tokens[9]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]));

      measurement               = Eigen::Isometry3d::Identity();
      measurement.translation() = translation;
      measurement.linear()      = rotation.toRotationMatrix();

      covariance = Eigen::Matrix<double, 6, 6>::Zero();
      size_t idx = 10;
      for (int row = 0; row < 6; ++row) {
        for (int col = row; col < 6; ++col) {
          covariance(row, col) = std::stod(tokens[idx++]);
          covariance(col, row) = covariance(row, col);
        }
      }
    }
  };

  /**
   * @brief Prior measurement (absolute pose constraint).
   */
  struct PriorMeas {
    double timestamp;
    Eigen::Isometry3d pose;
    Eigen::Matrix<double, 6, 6> covariance;

    PriorMeas() = default;

    explicit PriorMeas(const std::vector<std::string>& tokens) {
      timestamp = std::stod(tokens[1]);
      Eigen::Vector3d position(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));
      Eigen::Quaterniond orientation(std::stod(tokens[8]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]));

      pose               = Eigen::Isometry3d::Identity();
      pose.translation() = position;
      pose.linear()      = orientation.toRotationMatrix();

      covariance = Eigen::Matrix<double, 6, 6>::Zero();
      size_t idx = 9;
      for (int row = 0; row < 6; ++row) {
        for (int col = row; col < 6; ++col) {
          covariance(row, col) = std::stod(tokens[idx++]);
          covariance(col, row) = covariance(row, col);
        }
      }
    }
  };

  /**
   * @brief Landmark measurement (pose-to-landmark observation).
   */
  struct LandmarkMeas {
    double timestamp;
    size_t landmark_id;
    Eigen::Isometry3d measurement;
    Eigen::Matrix<double, 6, 6> covariance;

    explicit LandmarkMeas(const std::vector<std::string>& tokens) {
      timestamp   = std::stod(tokens[1]);
      landmark_id = std::stoul(tokens[2]);
      Eigen::Vector3d translation(std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]));
      Eigen::Quaterniond rotation(std::stod(tokens[9]), std::stod(tokens[6]), std::stod(tokens[7]), std::stod(tokens[8]));

      measurement               = Eigen::Isometry3d::Identity();
      measurement.translation() = translation;
      measurement.linear()      = rotation.toRotationMatrix();

      covariance = Eigen::Matrix<double, 6, 6>::Zero();
      size_t idx = 10;
      for (int row = 0; row < 6; ++row) {
        for (int col = row; col < 6; ++col) {
          covariance(row, col) = std::stod(tokens[idx++]);
          covariance(col, row) = covariance(row, col);
        }
      }
    }
  };

} // namespace examples
