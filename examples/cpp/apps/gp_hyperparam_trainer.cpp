/**
 * @file gp_hyperparam_trainer.cpp
 * @brief Train GP motion prior hyperparameters (Qc diagonal) from trajectory data.
 *
 * This tool computes the optimal Qc diagonal values for GP motion priors by
 * analyzing the acceleration statistics of a given trajectory:
 * 1. Loads poses from a g2o file
 * 2. Computes velocities between consecutive poses
 * 3. Computes accelerations from velocity changes
 * 4. Outputs mean and variance of accelerations (Qc_diag = variance)
 *
 * The output variance values can be used as qc_diag in the experiment config.
 *
 * Usage: ./gp_hyperparam_trainer <input.g2o>
 */

#include <fstream>

// Project utilities
#include "../common/data_types.h"
#include "../common/g2o_parser.h"
#include "../common/logger.h"

// gsolver library
#include <gsolver/maths/geometry3d.h>

using namespace gsolver;
using namespace examples;

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
  LOG_SET_MODULE("GPHyperparamTrainer");

  // ======================= Parse Command Line Arguments =======================
  if (argc < 2) {
    LOG_ERROR("Usage: {} <input.g2o>", argv[0]);
    LOG_INFO("  Analyzes trajectory to compute optimal qc_diag values for GP priors");
    return 1;
  }

  const std::string input_path = argv[1];

  if (!std::ifstream(input_path)) {
    LOG_ERROR("Cannot open input file: {}", input_path);
    return 1;
  }

  // ======================= Initialize =========================================
  LOG_SECTION("GP Hyperparameter Trainer (Qc)");

  // ======================= Load Data ==========================================
  LOG_STEP(1, 4, "Loading data from: {}", input_path);

  std::vector<PoseInit> poses;
  std::vector<OdometryMeas> odometry;
  std::vector<PriorMeas> priors;
  std::vector<LandmarkMeas> landmarks;
  parseG2OFile(input_path, poses, odometry, priors, landmarks);

  if (poses.size() < 3) {
    LOG_ERROR("Need at least 3 poses to compute accelerations, got {}", poses.size());
    return 1;
  }

  LOG_INFO("Found {} poses", poses.size());

  // ======================= Compute Velocities =================================
  LOG_STEP(2, 4, "Computing velocities from {} pose pairs...", poses.size() - 1);

  std::vector<Eigen::Matrix<double, 6, 1>> velocities;
  velocities.reserve(poses.size() - 1);

  for (size_t i = 0; i < poses.size() - 1; ++i) {
    const double dt                                = poses[i + 1].timestamp - poses[i].timestamp;
    const Eigen::Isometry3d pose_before            = poses[i].pose;
    const Eigen::Isometry3d pose_after             = poses[i + 1].pose;
    const Eigen::Isometry3d delta                  = pose_before.inverse() * pose_after;
    const Eigen::Matrix<double, 6, 1> delta_values = logMapSE3(delta);
    const Eigen::Matrix<double, 6, 1> velocity     = delta_values / dt;
    velocities.push_back(velocity);
  }

  LOG_INFO("Computed {} velocity vectors", velocities.size());

  // ======================= Compute Accelerations ==============================
  LOG_STEP(3, 4, "Computing accelerations from {} velocity pairs...", velocities.size() - 1);

  std::vector<Eigen::Matrix<double, 6, 1>> accelerations;
  accelerations.reserve(velocities.size() - 1);

  for (size_t i = 0; i < velocities.size() - 1; ++i) {
    const double dt                                   = poses[i + 1].timestamp - poses[i].timestamp;
    const Eigen::Isometry3d pose_before               = poses[i].pose;
    const Eigen::Isometry3d pose_after                = poses[i + 1].pose;
    const Eigen::Matrix<double, 6, 1> velocity_before = velocities[i];
    const Eigen::Matrix<double, 6, 1> velocity_after  = velocities[i + 1];

    // Adjoint transformation for velocity in body frame
    Eigen::Matrix3d R                       = pose_before.linear().transpose() * pose_after.linear();
    Eigen::Matrix<double, 6, 6> adj         = Eigen::Matrix<double, 6, 6>::Zero();
    adj.block<3, 3>(0, 0)                   = R;
    adj.block<3, 3>(3, 3)                   = R;
    const Eigen::Matrix<double, 6, 1> delta = adj * velocity_after - velocity_before;
    const Eigen::Matrix<double, 6, 1> accel = delta / dt;
    accelerations.push_back(accel);
  }

  LOG_INFO("Computed {} acceleration vectors", accelerations.size());

  // ======================= Compute Statistics =================================
  LOG_STEP(4, 4, "Computing acceleration statistics...");

  // Compute mean
  Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
  for (const auto& accel : accelerations) {
    mean += accel;
  }
  mean /= static_cast<double>(accelerations.size());

  // Compute variance (unbiased estimator with N-1)
  Eigen::Matrix<double, 6, 1> variance = Eigen::Matrix<double, 6, 1>::Zero();
  for (const auto& accel : accelerations) {
    Eigen::Matrix<double, 6, 1> diff = accel - mean;
    variance += diff.cwiseProduct(diff);
  }
  variance /= static_cast<double>(accelerations.size() - 1);

  // ======================= Output Results =====================================
  LOG_SECTION("Results");

  LOG_INFO("Acceleration mean:     [{}, {}, {}, {}, {}, {}]", mean(0), mean(1), mean(2), mean(3), mean(4), mean(5));
  LOG_INFO("Acceleration variance: [{}, {}, {}, {}, {}, {}]",
           variance(0),
           variance(1),
           variance(2),
           variance(3),
           variance(4),
           variance(5));
  LOG_INFO("Suggested qc_diag for config file:");
  LOG_INFO("  qc_diag: [{}, {}, {}, {}, {}, {}]", variance(0), variance(1), variance(2), variance(3), variance(4), variance(5));

  LOG_SECTION("Done");
  return 0;
}
