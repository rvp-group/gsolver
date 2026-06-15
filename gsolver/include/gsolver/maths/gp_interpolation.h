#pragma once

#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/derivatives.h>

namespace gsolver {

  struct GPInterpDistribution {
    double timestamp;
    Eigen::Isometry3d pose;
    Eigen::Vector<double, 6> vel;
    Eigen::Matrix3d covariance;
  };

  Eigen::Matrix<double, 12, 12> PHI(double ti, double tj) {
    double dt                         = ti - tj;
    Eigen::Matrix<double, 12, 12> phi = Eigen::Matrix<double, 12, 12>::Identity();
    phi.block<6, 6>(0, 6)             = dt * Eigen::Matrix<double, 6, 6>::Identity();
    return phi;
  }

  Eigen::Matrix<double, 12, 12> Q_i(const Eigen::Matrix<double, 6, 6>& Qc, double ti, double tj) {
    double dt                       = tj - ti;
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
    Q.block<6, 6>(0, 0)             = (dt * dt * dt / 3.0) * Qc;
    Q.block<6, 6>(0, 6)             = (dt * dt / 2.0) * Qc;
    Q.block<6, 6>(6, 0)             = (dt * dt / 2.0) * Qc;
    Q.block<6, 6>(6, 6)             = dt * Qc;
    return Q;
  }

  static std::vector<GPInterpDistribution>
  GPInterp(FactorGraph& factor_graph, Eigen::Vector<double, 6> Qc_diag, int hertz = 100) {
    std::vector<GPInterpDistribution> interp_results;

    std::vector<std::shared_ptr<VariableNodeBase>> pose_variable_nodes = factor_graph.getPoseVariableNodes();

    // Get the last timestamp
    std::shared_ptr<SE3PoseVel> last_variable_node = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes.back());
    const double T                                 = last_variable_node->timestamp_;

    // Create interpolation timestamps
    std::vector<double> interp_time;
    const int N = pose_variable_nodes.size();
    // double dt   = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[1])->timestamp_ -
    //             std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[0])->timestamp_;
    // double dt_interp = dt / hertz;
    // double t_curr    = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[0])->timestamp_;
    // while (t_curr <= T) {
    //   interp_time.push_back(t_curr);
    //   t_curr += dt_interp;
    // }

    double t_curr = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[0])->timestamp_;
    for (size_t i = 0; i < N - 1; ++i) {
      double current_dt = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[i + 1])->timestamp_ -
                          std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[i])->timestamp_;
      double current_dt_interp = current_dt / hertz;
      for (size_t j = 0; j < hertz; ++j) {
        interp_time.push_back(t_curr);
        t_curr += current_dt_interp;
      }
    }

    // Output vectors using Octave names
    std::vector<double> interp_x(interp_time.size());
    std::vector<double> interp_y(interp_time.size());
    std::vector<double> interp_z(interp_time.size());
    std::vector<double> interp_roll(interp_time.size());
    std::vector<double> interp_pitch(interp_time.size());
    std::vector<double> interp_yaw(interp_time.size());
    std::vector<double> interp_vx(interp_time.size());
    std::vector<double> interp_vy(interp_time.size());
    std::vector<double> interp_vz(interp_time.size());
    std::vector<double> interp_omega_x(interp_time.size());
    std::vector<double> interp_omega_y(interp_time.size());
    std::vector<double> interp_omega_z(interp_time.size());
    std::vector<double> sigma_x(interp_time.size());
    std::vector<double> sigma_y(interp_time.size());
    std::vector<double> sigma_z(interp_time.size());
    std::vector<Eigen::Isometry3d> T_tau_hat_vector(interp_time.size());

    Eigen::Matrix<double, 6, 6> Q_c = Qc_diag.asDiagonal();
    for (int i = 0; i < Q_c.rows(); ++i) {
      if (Q_c(i, i) < 1e-4) {
        Q_c(i, i) += 1e-4;
      }
    }

    for (size_t i = 0; i < interp_time.size(); ++i) {
      double tau = interp_time[i];

      // Find last index where time <= tau
      size_t idx = 0;
      for (size_t j = 0; j < N; ++j) {
        double time = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[j])->timestamp_;
        if (time <= tau) {
          idx = j;
        } else {
          break;
        }
      }

      Eigen::Isometry3d T_tau_hat;
      Vector6d omega_tau_hat;
      Eigen::Matrix<double, 12, 12> P_tau_hat = Eigen::Matrix<double, 12, 12>::Zero();

      double time_idx = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[idx])->timestamp_;
      if (tau < std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[0])->timestamp_) {
        // Extrapolation before first pose
        const std::shared_ptr<SE3PoseVel> variable_idx = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[0]);
        double t_i                                     = variable_idx->timestamp_;

        Eigen::Isometry3d x_i = variable_idx->mu_.pose;
        Vector6d v_i          = variable_idx->mu_.vel;

        T_tau_hat     = x_i * expMapSE3((tau - t_i) * v_i);
        omega_tau_hat = v_i;

        // TODO: Compute P_tau_hat for extrapolation before first pose
      } else if (idx == N - 1) {
        // Extrapolation
        const std::shared_ptr<SE3PoseVel> variable_idx = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[idx]);
        double t_i                                     = variable_idx->timestamp_;
        assert(t_i <= tau);

        Eigen::Isometry3d x_i = variable_idx->mu_.pose;
        Vector6d v_i          = variable_idx->mu_.vel;

        T_tau_hat     = x_i * expMapSE3((tau - t_i) * v_i);
        omega_tau_hat = v_i;

        Eigen::Matrix<double, 12, 12> F_t1     = DerivativeUtils::getJacKnot1(x_i, T_tau_hat, v_i, omega_tau_hat, tau - t_i);
        Eigen::Matrix<double, 12, 12> E_t1     = DerivativeUtils::getJacKnot2(x_i, T_tau_hat, v_i, omega_tau_hat, tau - t_i);
        Eigen::Matrix<double, 12, 12> E_t1_inv = E_t1.inverse();

        Eigen::Matrix<double, 12, 12> Q_i_tau = Q_i(Q_c, t_i, tau);
        Eigen::Matrix<double, 12, 12> P_end   = variable_idx->Sigma_;

        P_tau_hat = E_t1_inv * (F_t1 * P_end * F_t1.transpose() + Q_i_tau) * E_t1_inv.transpose();
      } else {
        // Interpolation
        const std::shared_ptr<SE3PoseVel> variable_i      = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[idx]);
        const std::shared_ptr<SE3PoseVel> variable_i_next = std::dynamic_pointer_cast<SE3PoseVel>(pose_variable_nodes[idx + 1]);
        double t_i                                        = variable_i->timestamp_;
        double t_i_next                                   = variable_i_next->timestamp_;

        Eigen::Isometry3d x_i      = variable_i->mu_.pose;
        Eigen::Isometry3d x_i_next = variable_i_next->mu_.pose;

        Vector6d v_i      = variable_i->mu_.vel;
        Vector6d v_i_next = variable_i_next->mu_.vel;

        auto Q_i_tau        = Q_i(Q_c, t_i, tau);
        auto Q_i_i_next     = Q_i(Q_c, t_i, t_i_next);
        auto PHI_tau_i      = PHI(tau, t_i);
        auto PHI_i_next_tau = PHI(t_i_next, tau);
        auto PHI_i_next_i   = PHI(t_i_next, t_i);

        auto PSI_tau    = Q_i_tau * PHI_i_next_tau.transpose() * Q_i_i_next.inverse();
        auto LAMBDA_tau = PHI_tau_i - PSI_tau * PHI_i_next_i;

        Eigen::Matrix<double, 6, 12> LAMBDA_tau_1 = LAMBDA_tau.topRows(6);
        Eigen::Matrix<double, 6, 12> LAMBDA_tau_2 = LAMBDA_tau.bottomRows(6);
        Eigen::Matrix<double, 6, 12> PSI_tau_1    = PSI_tau.topRows(6);
        Eigen::Matrix<double, 6, 12> PSI_tau_2    = PSI_tau.bottomRows(6);

        Eigen::Matrix<double, 12, 1> gamma_i_i_hat;
        gamma_i_i_hat << Vector6d::Zero(), v_i;

        Vector6d zeta_i_i_next = logMapSE3((x_i.inverse() * x_i_next));
        Eigen::Matrix3d R      = x_i.linear().transpose() * x_i_next.linear();
        Eigen::Matrix<double, 6, 6> adj;
        adj.setZero();
        adj.topLeftCorner<3, 3>()     = R;
        adj.bottomRightCorner<3, 3>() = R;
        Vector6d omega_i_i_next       = adj * v_i_next;

        Eigen::Matrix<double, 12, 1> gamma_i_i_next_hat;
        gamma_i_i_next_hat << zeta_i_i_next, omega_i_i_next;

        T_tau_hat = x_i * expMapSE3(LAMBDA_tau_1 * gamma_i_i_hat + PSI_tau_1 * gamma_i_i_next_hat);

        R = x_i.linear().transpose() * T_tau_hat.linear();
        adj.setZero();
        adj.topLeftCorner<3, 3>()     = R.transpose();
        adj.bottomRightCorner<3, 3>() = R.transpose();
        omega_tau_hat                 = adj * (LAMBDA_tau_2 * gamma_i_i_hat + PSI_tau_2 * gamma_i_i_next_hat);

        if (std::abs(t_i - tau) < 1e-6) {
          // tau is very close to t_i, avoid Q_i_tau singularity
          P_tau_hat = variable_i->Sigma_;
        } else if (std::abs(t_i_next - tau) < 1e-6) {
          // tau is very close to t_i_next, avoid Q_tau_i_next singularity
          P_tau_hat = variable_i_next->Sigma_;
        } else {
          auto F_t1 = DerivativeUtils::getJacKnot1(x_i, T_tau_hat, v_i, omega_tau_hat, tau - t_i);
          auto E_t1 = DerivativeUtils::getJacKnot2(x_i, T_tau_hat, v_i, omega_tau_hat, tau - t_i);
          auto F_2t = DerivativeUtils::getJacKnot1(T_tau_hat, x_i_next, omega_tau_hat, v_i_next, t_i_next - tau);
          auto E_2t = DerivativeUtils::getJacKnot2(T_tau_hat, x_i_next, omega_tau_hat, v_i_next, t_i_next - tau);

          auto Qt1_inv      = Q_i_tau.inverse();
          auto Q_tau_i_next = Q_i(Q_c, tau, t_i_next);
          auto Q2t_inv      = Q_tau_i_next.inverse();

          // auto P_1n2 = H_inv.block<24, 24>(12 * idx, 12 * idx);
          Eigen::Matrix<double, 24, 24> P_1n2 = Eigen::Matrix<double, 24, 24>::Zero();
          P_1n2.block<12, 12>(0, 0)           = variable_i->Sigma_;
          P_1n2.block<12, 12>(12, 12)         = variable_i_next->Sigma_;

          Eigen::Matrix<double, 24, 12> A;
          A << F_t1.transpose() * Qt1_inv * E_t1, E_2t.transpose() * Q2t_inv * F_2t;

          Eigen::Matrix<double, 24, 24> B = Eigen::Matrix<double, 24, 24>::Zero();
          B.block<12, 12>(0, 0)           = F_t1.transpose() * Qt1_inv * F_t1;
          B.block<12, 12>(12, 12)         = E_2t.transpose() * Q2t_inv * E_2t;

          auto F_21    = DerivativeUtils::getJacKnot1(x_i, x_i_next, v_i, v_i_next, t_i_next - t_i);
          auto E_21    = DerivativeUtils::getJacKnot2(x_i, x_i_next, v_i, v_i_next, t_i_next - t_i);
          auto Q21_inv = Q_i_i_next.inverse();

          Eigen::Matrix<double, 24, 24> Pinv_comp = Eigen::Matrix<double, 24, 24>::Zero();
          Pinv_comp.block<12, 12>(0, 0)           = F_21.transpose() * Q21_inv * F_21;
          Pinv_comp.block<12, 12>(12, 0)          = E_21.transpose() * Q21_inv * F_21;
          Pinv_comp.block<12, 12>(0, 12)          = Pinv_comp.block<12, 12>(12, 0).transpose();
          Pinv_comp.block<12, 12>(12, 12)         = E_21.transpose() * Q21_inv * E_21;

          // Eigen::Matrix<double, 12, 12> P_t_inv = E_t1.transpose() * Qt1_inv * E_t1 + F_2t.transpose() * Q2t_inv * F_2t -
          //                                         A.transpose() * ((P_1n2.inverse() + B - Pinv_comp).inverse()) * A;
          Eigen::Matrix<double, 12, 12> P_t_inv = E_t1.transpose() * Qt1_inv * E_t1 + F_2t.transpose() * Q2t_inv * F_2t -
                                                  A.transpose() * ((P_1n2.inverse() + B).inverse()) * A;

          P_tau_hat = P_t_inv.inverse();
        }
      }

      GPInterpDistribution new_interp = GPInterpDistribution();

      // pose
      new_interp.pose = T_tau_hat;

      // velocity
      new_interp.vel = omega_tau_hat;

      // covariance
      new_interp.covariance = P_tau_hat.block<3, 3>(0, 0);

      interp_results.push_back(new_interp);
    }

    return interp_results;
  }

  /**
   * @brief GP interpolation at specific query timestamps.
   *
   * Given a sorted list of SE3PoseVel knots and a set of query timestamps,
   * returns the GP-interpolated pose (and 3x3 position covariance) at each query time.
   * Timestamps outside the knot range are handled via extrapolation.
   *
   * @param pose_nodes Sorted SE3PoseVel nodes (ascending timestamp)
   * @param query_times Query timestamps (arbitrary order; results follow this order)
   * @param Qc_diag    Diagonal of the continuous-time power spectral density (6D)
   */
  static std::vector<GPInterpDistribution>
  GPInterpAtTimestamps(const std::vector<std::shared_ptr<SE3PoseVel>>& pose_nodes,
                       const std::vector<double>& query_times,
                       const Eigen::Vector<double, 6>& Qc_diag) {
    std::vector<GPInterpDistribution> results;
    results.reserve(query_times.size());

    const int N = static_cast<int>(pose_nodes.size());
    if (N == 0)
      return results;

    Eigen::Matrix<double, 6, 6> Q_c = Qc_diag.asDiagonal();
    for (int i = 0; i < 6; ++i) {
      if (Q_c(i, i) < 1e-4)
        Q_c(i, i) += 1e-4;
    }

    for (double tau : query_times) {
      size_t idx = 0;
      for (size_t j = 0; j < static_cast<size_t>(N); ++j) {
        if (pose_nodes[j]->timestamp_ <= tau)
          idx = j;
        else
          break;
      }

      Eigen::Isometry3d T_tau;
      Vector6d          omega_tau;
      Eigen::Matrix<double, 12, 12> P_tau = Eigen::Matrix<double, 12, 12>::Zero();

      if (tau < pose_nodes[0]->timestamp_) {
        // Extrapolation before first knot
        const auto& n   = pose_nodes[0];
        double      t_i = n->timestamp_;
        T_tau           = n->mu_.pose * expMapSE3((tau - t_i) * n->mu_.vel);
        omega_tau       = n->mu_.vel;
      } else if (idx == static_cast<size_t>(N - 1)) {
        // Extrapolation beyond last knot
        const auto& n   = pose_nodes[idx];
        double      t_i = n->timestamp_;
        T_tau           = n->mu_.pose * expMapSE3((tau - t_i) * n->mu_.vel);
        omega_tau       = n->mu_.vel;

        auto F  = DerivativeUtils::getJacKnot1(n->mu_.pose, T_tau, n->mu_.vel, omega_tau, tau - t_i);
        auto E  = DerivativeUtils::getJacKnot2(n->mu_.pose, T_tau, n->mu_.vel, omega_tau, tau - t_i);
        P_tau   = E.inverse() * (F * n->Sigma_ * F.transpose() + Q_i(Q_c, t_i, tau)) * E.inverse().transpose();
      } else {
        // Interpolation between knots idx and idx+1
        const auto& ni      = pose_nodes[idx];
        const auto& ni1     = pose_nodes[idx + 1];
        double      t_i     = ni->timestamp_;
        double      t_i_next = ni1->timestamp_;

        const Eigen::Isometry3d& x_i      = ni->mu_.pose;
        const Eigen::Isometry3d& x_i_next = ni1->mu_.pose;
        const Vector6d&          v_i      = ni->mu_.vel;
        const Vector6d&          v_i_next = ni1->mu_.vel;

        auto Q_i_tau        = Q_i(Q_c, t_i, tau);
        auto Q_i_i_next     = Q_i(Q_c, t_i, t_i_next);
        auto PHI_tau_i      = PHI(tau, t_i);
        auto PHI_i_next_tau = PHI(t_i_next, tau);
        auto PHI_i_next_i   = PHI(t_i_next, t_i);

        auto PSI_tau    = Q_i_tau * PHI_i_next_tau.transpose() * Q_i_i_next.inverse();
        auto LAMBDA_tau = PHI_tau_i - PSI_tau * PHI_i_next_i;

        Eigen::Matrix<double, 6, 12> L1 = LAMBDA_tau.topRows(6);
        Eigen::Matrix<double, 6, 12> L2 = LAMBDA_tau.bottomRows(6);
        Eigen::Matrix<double, 6, 12> P1 = PSI_tau.topRows(6);
        Eigen::Matrix<double, 6, 12> P2 = PSI_tau.bottomRows(6);

        Eigen::Matrix<double, 12, 1> gamma_i;
        gamma_i << Vector6d::Zero(), v_i;

        Vector6d          zeta       = logMapSE3(x_i.inverse() * x_i_next);
        Eigen::Matrix3d   R          = x_i.linear().transpose() * x_i_next.linear();
        Eigen::Matrix<double, 6, 6> adj;
        adj.setZero();
        adj.topLeftCorner<3, 3>()     = R;
        adj.bottomRightCorner<3, 3>() = R;
        Vector6d omega_i_next         = adj * v_i_next;

        Eigen::Matrix<double, 12, 1> gamma_i_next;
        gamma_i_next << zeta, omega_i_next;

        T_tau = x_i * expMapSE3(L1 * gamma_i + P1 * gamma_i_next);

        R = x_i.linear().transpose() * T_tau.linear();
        adj.setZero();
        adj.topLeftCorner<3, 3>()     = R.transpose();
        adj.bottomRightCorner<3, 3>() = R.transpose();
        omega_tau                     = adj * (L2 * gamma_i + P2 * gamma_i_next);

        if (std::abs(t_i - tau) < 1e-6) {
          P_tau = ni->Sigma_;
        } else if (std::abs(t_i_next - tau) < 1e-6) {
          P_tau = ni1->Sigma_;
        } else {
          auto F_t1 = DerivativeUtils::getJacKnot1(x_i, T_tau, v_i, omega_tau, tau - t_i);
          auto E_t1 = DerivativeUtils::getJacKnot2(x_i, T_tau, v_i, omega_tau, tau - t_i);
          auto F_2t = DerivativeUtils::getJacKnot1(T_tau, x_i_next, omega_tau, v_i_next, t_i_next - tau);
          auto E_2t = DerivativeUtils::getJacKnot2(T_tau, x_i_next, omega_tau, v_i_next, t_i_next - tau);

          auto Qt1_inv      = Q_i_tau.inverse();
          auto Q_tau_i_next = Q_i(Q_c, tau, t_i_next);
          auto Q2t_inv      = Q_tau_i_next.inverse();

          Eigen::Matrix<double, 24, 24> P_1n2 = Eigen::Matrix<double, 24, 24>::Zero();
          P_1n2.block<12, 12>(0, 0)           = ni->Sigma_;
          P_1n2.block<12, 12>(12, 12)         = ni1->Sigma_;

          Eigen::Matrix<double, 24, 12> A;
          A << F_t1.transpose() * Qt1_inv * E_t1, E_2t.transpose() * Q2t_inv * F_2t;

          Eigen::Matrix<double, 24, 24> B = Eigen::Matrix<double, 24, 24>::Zero();
          B.block<12, 12>(0, 0)           = F_t1.transpose() * Qt1_inv * F_t1;
          B.block<12, 12>(12, 12)         = E_2t.transpose() * Q2t_inv * E_2t;

          Eigen::Matrix<double, 12, 12> P_t_inv =
            E_t1.transpose() * Qt1_inv * E_t1 + F_2t.transpose() * Q2t_inv * F_2t -
            A.transpose() * (P_1n2.inverse() + B).inverse() * A;

          P_tau = P_t_inv.inverse();
        }
      }

      GPInterpDistribution dist;
      dist.timestamp  = tau;
      dist.pose       = T_tau;
      dist.vel        = omega_tau;
      dist.covariance = P_tau.block<3, 3>(0, 0);
      results.push_back(dist);
    }

    return results;
  }

} // namespace gsolver
