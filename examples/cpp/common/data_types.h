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

  // ============================================================================
  // Stereo SLAM Data Types
  // ============================================================================

  /**
   * @brief Camera intrinsic parameters for a stereo rig.
   *
   * g2o token: CAMERA_PARAMS fx fy cx cy bf
   * where bf = baseline * fx.
   */
  struct CameraParams {
    double fx = 1.0, fy = 1.0, cx = 0.0, cy = 0.0, bf = 0.1;

    CameraParams() = default;

    explicit CameraParams(const std::vector<std::string>& tokens) {
      fx = std::stod(tokens[1]);
      fy = std::stod(tokens[2]);
      cx = std::stod(tokens[3]);
      cy = std::stod(tokens[4]);
      bf = std::stod(tokens[5]);
    }

    Eigen::Matrix<double, 3, 4> projectionMatrix() const {
      Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Zero();
      P(0, 0) = fx;
      P(0, 2) = cx;
      P(1, 1) = fy;
      P(1, 2) = cy;
      P(2, 2) = 1.0;
      return P;
    }

    double baseline() const { return bf / fx; }
  };

  /**
   * @brief Initial 3D landmark position parsed from g2o file.
   *
   * g2o token: LANDMARK_3D id x y z cov(6 upper-triangular)
   */
  struct LandmarkInit {
    size_t landmark_id = 0;
    Eigen::Vector3d position;
    Eigen::Matrix<double, 3, 3> covariance;

    explicit LandmarkInit(const std::vector<std::string>& tokens) {
      landmark_id = std::stoul(tokens[1]);
      position    = Eigen::Vector3d(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));
      covariance  = Eigen::Matrix<double, 3, 3>::Zero();
      size_t idx  = 5;
      for (int row = 0; row < 3; ++row)
        for (int col = row; col < 3; ++col) {
          covariance(row, col) = std::stod(tokens[idx++]);
          covariance(col, row) = covariance(row, col);
        }
    }
  };

  /**
   * @brief Stereo camera observation linking a pose to a 3D landmark.
   *
   * g2o token: EDGE_STEREO pose_timestamp landmark_id u_left v u_right information
   * Observation stored as [u_left, v, u_right]; disparity = u_left - u_right.
   */
  struct StereoMeas {
    double timestamp   = 0.0;
    size_t landmark_id = 0;
    Eigen::Vector3d observation;            // [u_left, v, u_right]
    Eigen::Matrix<double, 3, 3> information;

    explicit StereoMeas(const std::vector<std::string>& tokens) {
      timestamp   = std::stod(tokens[1]);
      landmark_id = std::stoul(tokens[2]);
      observation = Eigen::Vector3d(std::stod(tokens[3]), std::stod(tokens[4]), std::stod(tokens[5]));
      information = Eigen::Matrix<double, 3, 3>::Zero();
      double info = std::stod(tokens[6]);
      information(0, 0) = info;
      information(1, 1) = info;
      information(2, 2) = info;
    }

    Eigen::Matrix<double, 3, 3> covariance() const { return information.inverse(); }

    // Returns [u_left, v, disparity] — the measurement model uses disparity
    Eigen::Vector3d stereoObservation() const {
      return Eigen::Vector3d(observation(0), observation(1), observation(0) - observation(2));
    }
  };

} // namespace examples
