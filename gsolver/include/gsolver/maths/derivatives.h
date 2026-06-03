#pragma once

#include <Eigen/Dense>

#include <gsolver/maths/geometry3d.h>

namespace gsolver {

  class DerivativeUtils {
  public:
    using JacobianSE3vel       = Eigen::Matrix<double, 12, 12>;
    using JacobianSE3velDelta  = Eigen::Matrix<double, 6, 24>;
    using JacobianSE3velPrior  = Eigen::Matrix<double, 6, 12>;
    using JacobianSE3velGp     = Eigen::Matrix<double, 12, 24>;
    using TangentVectorSE3vel  = Eigen::Vector<double, 12>;
    using TangentVectorSE3     = Eigen::Vector<double, 6>;
    using Isometry3dDerivative = Eigen::Matrix<double, 4, 4>;
    using Matrix3d             = Eigen::Matrix<double, 3, 3>;
    using JacobianSE3          = Eigen::Matrix<double, 6, 6>;

    static JacobianSE3velPrior compute_se3vel_prior_jacobian(Isometry3d x, Isometry3d z_T) {
      Isometry3d I          = Isometry3d::Identity();
      JacobianSE3velPrior J = JacobianSE3velPrior::Zero();
      J.block<6, 6>(0, 0)   = jacob_dDinvP1invP2_de1e2(z_T.inverse(), I, x, 2);
      return J;
    }

    static Eigen::Matrix<double, 12, 24> compute_se3vel_gp_jacobian(const Isometry3d& T_i,
                                                                    const Isometry3d& T_i_next,
                                                                    const Vector6d& v_i,
                                                                    const Vector6d& v_i_next,
                                                                    const double& dt) {
      Eigen::Matrix<double, 12, 24> J = Eigen::Matrix<double, 12, 24>::Zero();
      const Isometry3d int_v_i_inv    = expMapSE3(dt * v_i).inverse();

      J.block<6, 6>(0, 0) = jacob_dDinvP1invP2_de1e2(int_v_i_inv, T_i, T_i_next, 1);
      J.block<6, 6>(0, 6) = jacob_dDinvP1invP2_de1e2_gp(int_v_i_inv, T_i, T_i_next, dt);
      J.block<3, 3>(6, 3) = skew(T_i.linear().transpose() * T_i_next.linear() * v_i_next.head(3));
      J.block<3, 3>(9, 3) = skew(T_i.linear().transpose() * T_i_next.linear() * v_i_next.tail(3));
      J.block<6, 6>(6, 6) = -Eigen::Matrix<double, 6, 6>::Identity(6, 6);

      J.block<6, 6>(0, 12)     = jacob_dDinvP1invP2_de1e2(int_v_i_inv, T_i, T_i_next, 2);
      J.block<3, 3>(6, 12 + 3) = -T_i.linear().transpose() * T_i_next.linear() * skew(v_i_next.head(3));
      J.block<3, 3>(9, 12 + 3) = -T_i.linear().transpose() * T_i_next.linear() * skew(v_i_next.tail(3));

      Eigen::Matrix3d R               = T_i.linear().transpose() * T_i_next.linear();
      Eigen::Matrix<double, 6, 6> adj = Eigen::Matrix<double, 6, 6>::Zero();
      adj.block(0, 0, 3, 3)           = R;
      adj.block(3, 3, 3, 3)           = R;
      J.block<6, 6>(6, 12 + 6)        = adj;
      return J;
    }

    static Eigen::Matrix<double, 12, 12> getJacKnot1(const Isometry3d& T_i,
                                                     const Isometry3d& T_i_next,
                                                     const Vector6d& v_i,
                                                     const Vector6d& v_i_next,
                                                     const double& dt) {
      Eigen::Matrix<double, 12, 12> J = Eigen::Matrix<double, 12, 12>::Zero();
      const Isometry3d int_v_i_inv    = expMapSE3(dt * v_i).inverse();

      J.block<6, 6>(0, 0) = jacob_dDinvP1invP2_de1e2(int_v_i_inv, T_i, T_i_next, 1);
      J.block<6, 6>(0, 6) = jacob_dDinvP1invP2_de1e2_gp(int_v_i_inv, T_i, T_i_next, dt);
      J.block<3, 3>(6, 3) = skew(T_i.linear().transpose() * T_i_next.linear() * v_i_next.head(3));
      J.block<3, 3>(9, 3) = skew(T_i.linear().transpose() * T_i_next.linear() * v_i_next.tail(3));
      J.block<6, 6>(6, 6) = -Eigen::Matrix<double, 6, 6>::Identity(6, 6);

      return J;
    }

    static Eigen::Matrix<double, 12, 12> getJacKnot2(const Isometry3d& T_i,
                                                     const Isometry3d& T_i_next,
                                                     const Vector6d& v_i,
                                                     const Vector6d& v_i_next,
                                                     const double& dt) {
      Eigen::Matrix<double, 12, 12> J = Eigen::Matrix<double, 12, 12>::Zero();
      const Isometry3d int_v_i_inv    = expMapSE3(dt * v_i).inverse();

      J.block<6, 6>(0, 0) = jacob_dDinvP1invP2_de1e2(int_v_i_inv, T_i, T_i_next, 2);
      J.block<3, 3>(6, 3) = -T_i.linear().transpose() * T_i_next.linear() * skew(v_i_next.head(3));
      J.block<3, 3>(9, 3) = -T_i.linear().transpose() * T_i_next.linear() * skew(v_i_next.tail(3));

      Eigen::Matrix3d R               = T_i.linear().transpose() * T_i_next.linear();
      Eigen::Matrix<double, 6, 6> adj = Eigen::Matrix<double, 6, 6>::Zero();
      adj.block(0, 0, 3, 3)           = R;
      adj.block(3, 3, 3, 3)           = R;
      J.block<6, 6>(6, 6)             = adj;
      return J;
    }

    static Eigen::Matrix<double, 6, 6>
    jacob_dDinvP1invP2_de1e2_gp(const Isometry3d& Dinv, const Isometry3d& P1, const Isometry3d& P2, double dt) {
      const Isometry3d P1inv       = P1.inverse();
      const Isometry3d DinvP1invP2 = Dinv * P1inv * P2;

      Eigen::Matrix<double, 6, 12> dLnT_dT = jacob_dlogv_dv(DinvP1invP2); // 6x12

      Eigen::Matrix<double, 12, 12> J1a = jacob_dAB_dA(P1inv * P2); // 12x12
      // Eigen::Matrix<double, 12, 12> J1a = jacob_dAB_dA(Dinv, P1inv * P2); // 12x12
      Eigen::Matrix<double, 12, 6> J1b = -dt * jacob_dDexpe_de(Dinv); // 12x6
      Eigen::Matrix<double, 6, 6> J1   = dLnT_dT * J1a * J1b;         // 6x6

      return J1;
    }

    static JacobianSE3velDelta compute_se3vel_delta_jacobian(Isometry3d xi, Isometry3d xj, Isometry3d z_T) {
      JacobianSE3velDelta J = Eigen::Matrix<double, 6, 24>::Zero();

      J.block(0, 0, 6, 6)  = jacob_dDinvP1invP2_de1e2(z_T.inverse(), xi, xj, 1);
      J.block(0, 12, 6, 6) = jacob_dDinvP1invP2_de1e2(z_T.inverse(), xi, xj, 2);

      return J;
    }

    static Eigen::Matrix<double, 6, 18> compute_se3vel_se3_jacobian(Isometry3d xi, Isometry3d xj, Isometry3d z_T) {
      Eigen::Matrix<double, 6, 18> J = Eigen::Matrix<double, 6, 18>::Zero();

      J.block(0, 0, 6, 6)  = jacob_dDinvP1invP2_de1e2(z_T.inverse(), xi, xj, 1);
      J.block(0, 12, 6, 6) = jacob_dDinvP1invP2_de1e2(z_T.inverse(), xi, xj, 2);

      return J;
    }

    static Eigen::Matrix<double, 6, 6>
    jacob_dDinvP1invP2_de1e2(const Isometry3d& Dinv, const Isometry3d& P1, const Isometry3d& P2, int one_or_two) {
      Isometry3d P1inv       = P1.inverse();
      Isometry3d DinvP1invP2 = Dinv * P1inv * P2;

      Eigen::Matrix<double, 6, 12> dLnT_dT = jacob_dlogv_dv(DinvP1invP2); // 6x12

      if (one_or_two == 1) {
        Eigen::Matrix<double, 12, 12> J1a = jacob_dAB_dA(P1inv * P2); // 12x12
        // Eigen::Matrix<double, 12, 12> J1a = jacob_dAB_dA(Dinv, P1inv * P2); // 12x12
        Eigen::Matrix<double, 12, 6> J1b = -jacob_dDexpe_de(Dinv); // 12x6
        Eigen::Matrix<double, 6, 6> J1   = dLnT_dT * J1a * J1b;    // 6x6

        return J1;
      }
      if (one_or_two == 2) {
        Eigen::Matrix<double, 12, 6> dAe_de = jacob_dDexpe_de(DinvP1invP2); // 12x6
        Eigen::Matrix<double, 6, 6> J2      = dLnT_dT * dAe_de;             // 6x6

        return J2;
      }
      return Eigen::Matrix<double, 6, 6>::Zero();
    }

    static Eigen::Matrix<double, 6, 12> jacob_dlogv_dv(const Isometry3d& P) {
      Eigen::Matrix<double, 6, 12> J = Eigen::Matrix<double, 6, 12>::Zero();
      Matrix3d R                     = P.linear();
      J.block<3, 9>(3, 0)            = jacob_dlogv_dv_SO3(R); // 3x9
      J(0, 9)                        = 1;
      J(1, 10)                       = 1;
      J(2, 11)                       = 1;
      return J;
    }

    static Eigen::Matrix<double, 3, 9> jacob_dlogv_dv_SO3(const Matrix3d& R) {
      double d          = 0.5 * (R.trace() - 1);
      Eigen::Vector3d a = Eigen::Vector3d::Zero();
      Matrix3d B        = -0.5 * Matrix3d::Identity();

      if (d <= 0.99999) {
        double theta = acos(d);
        double d2    = d * d;
        double sq    = sqrt(1 - d2);
        a            = vee_RmRt(R) * ((d * theta - sq) / (4 * pow(sq, 3))); // 3x1
        B            = -theta / (2 * sq) * Matrix3d::Identity();            // 3x3
      }

      return M3x9(a, B); // 3x9
    }

    static Eigen::Vector3d vee_RmRt(const Matrix3d& R) {
      Eigen::Vector3d v;
      v(0) = R(2, 1) - R(1, 2);
      v(1) = R(0, 2) - R(2, 0);
      v(2) = R(1, 0) - R(0, 1);
      return v;
    }

    static Eigen::Matrix<double, 3, 9> M3x9(const Eigen::Vector3d& a, const Matrix3d& B) {
      Eigen::Matrix<double, 3, 9> RES(3, 9);
      RES << a(0), -B(0, 2), B(0, 1), B(0, 2), a(0), -B(0, 0), -B(0, 1), B(0, 0), a(0), a(1), -B(1, 2), B(1, 1), B(1, 2), a(1),
        -B(1, 0), -B(1, 1), B(1, 0), a(1), a(2), -B(2, 2), B(2, 1), B(2, 2), a(2), -B(2, 0), -B(2, 1), B(2, 0), a(2);
      return RES;
    }

    static Eigen::Matrix<double, 12, 12> jacob_dAB_dA(const Isometry3d& B) {
      Eigen::Matrix<double, 12, 12> J  = Eigen::Matrix<double, 12, 12>::Zero();
      Eigen::Matrix<double, 4, 4> B_HM = B.matrix().transpose();
      for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
          for (int q = 0; q < 3; ++q) {
            J(r * 3 + q, c * 3 + q) = B_HM(r, c);
          }
        }
      }
      return J;
    }

    static Eigen::Matrix<double, 12, 6> jacob_dDexpe_de(const Isometry3d& D) {
      Eigen::Matrix<double, 12, 6> jacob = Eigen::Matrix<double, 12, 6>::Zero();
      Eigen::Matrix<double, 3, 3> dRot   = D.linear();
      jacob.block<3, 3>(9, 0)            = dRot;

      jacob.block<3, 1>(3, 5) = -dRot.col(0);
      jacob.block<3, 1>(6, 4) = dRot.col(0);

      jacob.block<3, 1>(0, 5) = dRot.col(1);
      jacob.block<3, 1>(6, 3) = -dRot.col(1);

      jacob.block<3, 1>(0, 4) = -dRot.col(2);
      jacob.block<3, 1>(3, 3) = dRot.col(2);

      return jacob;
    }
  };

} // namespace gsolver