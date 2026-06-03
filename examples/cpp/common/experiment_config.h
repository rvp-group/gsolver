/**
 * @file experiment_config.h
 * @brief Configuration loader for experiment parameters.
 *
 * Loads optimized hyperparameters from YAML configuration file
 * for different trajectory types (printing_room, sphere, torus, helix).
 */

#pragma once

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace examples {

  /**
   * @brief Experiment configuration parameters.
   */
  struct ExperimentParams {
    std::string description;
    Eigen::Vector<double, 6> qc_diag;
    int num_iterations;
  };

  /**
   * @brief Load experiment parameters from YAML config file.
   *
   * @param config_path Path to the YAML configuration file
   * @param trajectory_type Type of trajectory ("printing_room", "sphere", "torus", "helix")
   * @return ExperimentParams Loaded parameters (falls back to "default" if type not found)
   */
  inline ExperimentParams loadExperimentParams(const std::string& config_path, const std::string& trajectory_type) {
    ExperimentParams params;

    try {
      YAML::Node config = YAML::LoadFile(config_path);

      // Try to load specified trajectory type, fall back to default
      std::string type = trajectory_type;
      if (!config[type]) {
        std::cerr << "[Config] Warning: Unknown trajectory type '" << type << "', using 'default'" << std::endl;
        type = "default";
      }

      YAML::Node traj_config = config[type];

      // Load description
      params.description = traj_config["description"].as<std::string>("No description");

      // Load Qc diagonal
      auto qc_vec = traj_config["qc_diag"].as<std::vector<double>>();
      if (qc_vec.size() == 6) {
        params.qc_diag = Eigen::Map<Eigen::Vector<double, 6>>(qc_vec.data());
      } else {
        std::cerr << "[Config] Error: qc_diag must have 6 elements, using defaults" << std::endl;
        params.qc_diag = Eigen::Vector<double, 6>::Ones();
      }

      // Load number of iterations
      params.num_iterations = traj_config["num_iterations"].as<int>(50);

    } catch (const YAML::Exception& e) {
      std::cerr << "[Config] Error loading config: " << e.what() << std::endl;
      std::cerr << "[Config] Using default parameters" << std::endl;
      params.description    = "Default (config load failed)";
      params.qc_diag        = Eigen::Vector<double, 6>::Ones();
      params.num_iterations = 50;
    }

    return params;
  }

  /**
   * @brief Get the default config file path relative to executable.
   *
   * Tries multiple paths to find the config file.
   */
  inline std::string getDefaultConfigPath() {
    // Try different relative paths depending on where executable is run from
    std::vector<std::string> paths = {
      "config/experiment_params.yaml",                   // From examples/cpp/
      "../../../examples/config/experiment_params.yaml", // From build/examples/cpp/
      "examples/config/experiment_params.yaml",          // From project root
      "../examples/config/experiment_params.yaml",       // From build/
    };

    for (const auto& path : paths) {
      if (std::ifstream(path).good()) {
        return path;
      }
    }

    return paths[1]; // Default fallback
  }

  /**
   * @brief Print experiment parameters to stdout.
   */
  inline void printParams(const ExperimentParams& params) {
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Description: " << params.description << std::endl;
    std::cout << "  Qc_diag: [" << params.qc_diag.transpose() << "]" << std::endl;
    std::cout << "  Iterations: " << params.num_iterations << std::endl;
  }

} // namespace examples
