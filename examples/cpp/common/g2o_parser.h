/**
 * @file g2o_parser.h
 * @brief Parser for G2O pose graph files.
 */

#pragma once

#include "data_types.h"
#include "logger.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace examples {

  /**
   * @brief Parse a G2O file and extract poses, odometry, priors, and landmarks.
   *
   * Supported entry types:
   * - VERTEX_SE3: Initial pose guesses
   * - EDGE_POSE_POSE_SE3: Odometry measurements
   * - EDGE_PRIOR_SE3: Prior measurements
   * - EDGE_POSE_LANDMARK_SE3: Landmark measurements
   *
   * @param filename Path to the G2O file
   * @param poses Output vector of initial poses
   * @param odometry Output vector of odometry measurements
   * @param priors Output vector of prior measurements
   * @param landmarks Output vector of landmark measurements
   */
  inline void parseG2OFile(const std::string& filename,
                           std::vector<PoseInit>& poses,
                           std::vector<OdometryMeas>& odometry,
                           std::vector<PriorMeas>& priors,
                           std::vector<LandmarkMeas>& landmarks) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      LOG_ERROR("Could not open file: {}", filename);
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream ss(line);
      std::vector<std::string> tokens;
      std::string token;
      while (ss >> token) {
        tokens.push_back(token);
      }

      if (tokens.empty())
        continue;

      if (tokens[0] == "VERTEX_SE3") {
        poses.emplace_back(tokens);
      } else if (tokens[0] == "EDGE_POSE_POSE_SE3") {
        odometry.emplace_back(tokens);
      } else if (tokens[0] == "EDGE_PRIOR_SE3") {
        priors.emplace_back(tokens);
      } else if (tokens[0] == "EDGE_POSE_LANDMARK_SE3") {
        landmarks.emplace_back(tokens);
      }
    }

    LOG_DEBUG(
      "Parsed {} poses, {} odometry, {} priors, {} landmarks", poses.size(), odometry.size(), priors.size(), landmarks.size());
  }

  /**
   * @brief Overload without landmarks for simpler PGO cases.
   */
  inline void parseG2OFile(const std::string& filename,
                           std::vector<PoseInit>& poses,
                           std::vector<OdometryMeas>& odometry,
                           std::vector<PriorMeas>& priors) {
    std::vector<LandmarkMeas> landmarks;
    parseG2OFile(filename, poses, odometry, priors, landmarks);
  }

  /**
   * @brief Parse a stereo-SLAM G2O file.
   *
   * Supported tokens:
   * - CAMERA_PARAMS : stereo camera intrinsics
   * - VERTEX_SE3    : initial pose guesses
   * - LANDMARK_3D   : 3D landmark positions
   * - EDGE_STEREO   : stereo pixel observations
   * - EDGE_PRIOR_SE3: absolute pose priors
   *
   * @param filename  Path to the G2O file
   * @param camera    Output camera parameters
   * @param poses     Output initial poses
   * @param landmarks Output initial landmark positions
   * @param stereo    Output stereo observations
   * @param priors    Output absolute pose priors
   */
  inline void parseStereoG2OFile(const std::string& filename,
                                 CameraParams& camera,
                                 std::vector<PoseInit>& poses,
                                 std::vector<LandmarkInit>& landmarks,
                                 std::vector<StereoMeas>& stereo,
                                 std::vector<PriorMeas>& priors) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      LOG_ERROR("Could not open stereo g2o file: {}", filename);
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream ss(line);
      std::vector<std::string> tokens;
      std::string token;
      while (ss >> token)
        tokens.push_back(token);

      if (tokens.empty())
        continue;

      if (tokens[0] == "CAMERA_PARAMS") {
        camera = CameraParams(tokens);
      } else if (tokens[0] == "VERTEX_SE3") {
        poses.emplace_back(tokens);
      } else if (tokens[0] == "LANDMARK_3D") {
        landmarks.emplace_back(tokens);
      } else if (tokens[0] == "EDGE_STEREO") {
        stereo.emplace_back(tokens);
      } else if (tokens[0] == "EDGE_PRIOR_SE3") {
        priors.emplace_back(tokens);
      }
    }

    LOG_DEBUG("Stereo g2o: {} poses, {} landmarks, {} observations, {} priors",
              poses.size(), landmarks.size(), stereo.size(), priors.size());
    LOG_DEBUG("Camera: fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f} baseline={:.4f}",
              camera.fx, camera.fy, camera.cx, camera.cy, camera.baseline());
  }

} // namespace examples
