#pragma once

#include <Eigen/Dense>

namespace gsolver {

  using Isometry3d = Eigen::Isometry3d;
  using Vector6d   = Eigen::Vector<double, 6>;

  //! rotation matrix in x
  inline Eigen::Matrix3d rotationX(const double& angle_) {
    Eigen::Matrix3d R;
    const double s = sin(angle_);
    const double c = cos(angle_);
    R << double(1.), double(0.), double(0.), double(0.), c, -s, double(0.), s, c;
    return R;
  }

  //! rotation matrix in y
  inline Eigen::Matrix3d rotationY(const double& angle_) {
    Eigen::Matrix3d R;
    const double s = sin(angle_);
    const double c = cos(angle_);
    R << c, double(0.), s, double(0.), double(1.), double(0.), -s, double(0.), c;
    return R;
  }

  //! rotation matrix in z
  inline Eigen::Matrix3d rotationZ(const double& angle) {
    Eigen::Matrix3d R;
    const double s = sin(angle);
    const double c = cos(angle);
    R << c, -s, double(0.), s, c, double(0.), double(0.), double(0.), double(1.);
    return R;
  }

  //! @brief flatten isometry by cols
  inline Eigen::Vector<double, 12> flattenByCols(const Isometry3d& t_) {
    Eigen::Vector<double, 12> v  = Eigen::Vector<double, 12>::Zero(12);
    v.template block<3, 1>(0, 0) = t_.matrix().template block<3, 1>(0, 0);
    v.template block<3, 1>(3, 0) = t_.matrix().template block<3, 1>(0, 1);
    v.template block<3, 1>(6, 0) = t_.matrix().template block<3, 1>(0, 2);
    v.template block<3, 1>(9, 0) = t_.matrix().template block<3, 1>(0, 3);
    return v;
  }

  //! @brief flatten isometry derivative by cols
  inline Eigen::Vector<double, 12> flattenByCols(const Eigen::Matrix<double, 4, 4>& t_) {
    Eigen::Vector<double, 12> v  = Eigen::Vector<double, 12>::Zero(12);
    v.template block<3, 1>(0, 0) = t_.matrix().template block<3, 1>(0, 0);
    v.template block<3, 1>(3, 0) = t_.matrix().template block<3, 1>(0, 1);
    v.template block<3, 1>(6, 0) = t_.matrix().template block<3, 1>(0, 2);
    v.template block<3, 1>(9, 0) = t_.matrix().template block<3, 1>(0, 3);
    return v;
  }

  inline Isometry3d fromFlattenByCols(const Eigen::Vector<double, 12>& v_, const bool reconditionate_rotation_ = true) {
    Isometry3d t                          = Isometry3d::Identity();
    t.matrix().template block<3, 1>(0, 0) = v_.template block<3, 1>(0, 0);
    t.matrix().template block<3, 1>(0, 1) = v_.template block<3, 1>(3, 0);
    t.matrix().template block<3, 1>(0, 2) = v_.template block<3, 1>(6, 0);
    t.matrix().template block<3, 1>(0, 3) = v_.template block<3, 1>(9, 0);

    if (reconditionate_rotation_) {
      const Eigen::Matrix3d& R = t.linear();
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::Matrix3d R_enforced = svd.matrixU() * svd.matrixV().transpose();
      t.linear()                 = R_enforced;
    }
    return t;
  }

  inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return S;
  }

  inline Eigen::Matrix3d expMapSO3(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d R;
    const double theta_square = omega.dot(omega);
    const double theta        = sqrt(theta_square);
    const Eigen::Matrix3d W   = skew(omega);
    const Eigen::Matrix3d K   = W / theta;
    if (theta_square < 1e-8) {
      R = Eigen::Matrix3d::Identity() + W;
    } else {
      const double one_minus_cos = 2.0 * sin(theta / 2.0) * sin(theta / 2.0);
      R                          = Eigen::Matrix3d::Identity() + sin(theta) * K + one_minus_cos * K * K;
    }
    return R;
  }

  inline Eigen::Vector3d logMapSO3(const Eigen::Matrix3d& R) {
    const double &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
    const double &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
    const double &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

    const double tr = R.trace();
    const double pi(M_PI);
    const double two(2);

    Eigen::Vector3d omega;
    if (tr + 1.0 < 1e-10) {
      if (abs(R33 + 1.0) > 1e-5) {
        omega = (pi / sqrt(two + two * R33)) * Eigen::Vector3d(R13, R23, 1.0 + R33);
      } else if (abs(R22 + 1.0) > 1e-5) {
        omega = (pi / sqrt(two + two * R22)) * Eigen::Vector3d(R12, 1.0 + R22, R32);
      } else {
        omega = (pi / sqrt(two + two * R11)) * Eigen::Vector3d(1.0 + R11, R21, R31);
      }
    } else {
      double magnitude;
      const double tr_3 = tr - 3.0;
      if (tr_3 < -1e-7) {
        double theta = acos((tr - 1.0) / two);
        magnitude    = theta / (two * sin(theta));
      } else {
        magnitude = 0.5 - tr_3 * tr_3 / 12.0;
      }
      omega = magnitude * Eigen::Vector3d(R32 - R23, R13 - R31, R21 - R12);
    }
    return omega;
  }

  inline Isometry3d expMapSE3(const Vector6d& x) {
    Eigen::Vector3d t = x.head(3);
    Eigen::Matrix3d R = expMapSO3(x.tail(3));
    Isometry3d T      = Isometry3d::Identity();
    T.translation()   = t;
    T.linear()        = R;
    return T;
  }

  inline Vector6d logMapSE3(const Isometry3d& T) {
    Vector6d x;
    x.head(3) = T.translation();
    x.tail(3) = logMapSO3(T.linear());
    return x;
  }

  // ---- small helpers ----
  inline double sq(double x) {
    return x * x;
  }

  inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
  }

  inline Eigen::Vector3d vee(const Eigen::Matrix3d& M) {
    return Eigen::Vector3d(M(2, 1), M(0, 2), M(1, 0));
  }

  // ---- Adjoint of SE3 ----
  inline Eigen::Matrix<double, 6, 6> adjoint_SE3(const Eigen::Isometry3d& T) {
    Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d R              = T.linear();
    Eigen::Vector3d p              = T.translation();
    Eigen::Matrix3d P              = hat(p);
    Ad.block<3, 3>(0, 0)           = R;
    Ad.block<3, 3>(0, 3)           = P * R;
    Ad.block<3, 3>(3, 3)           = R;
    return Ad;
  }

} // namespace gsolver
