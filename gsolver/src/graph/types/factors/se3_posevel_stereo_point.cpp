#include <gsolver/graph/types/factors/se3_posevel_stereo_point.h>

namespace gsolver {

  SE3PoseVelStereoPoint::SE3PoseVelStereoPoint(std::string id,
                                               const Eigen::Vector3d& z,
                                               const Eigen::Matrix<double, 3, 4>& P,
                                               double baseline,
                                               MatrixErrorDim Sigma)
    : FactorNode(id, Sigma), z_(z), P_(P), baseline_(baseline) {}

  void SE3PoseVelStereoPoint::updateErrorAndJacobian() {
    std::shared_ptr<Edge<SE3PoseVel>> edge_pose  = edges_.at<0>();
    std::shared_ptr<Edge<SE3Point>>  edge_point  = edges_.at<1>();
    const SE3PoseVel::EstimateType& x_posevel    = edge_pose->variable_to_factor_message_.mu_;
    const SE3Point::EstimateType&   x_point      = edge_point->variable_to_factor_message_.mu_;

    const Eigen::Isometry3d& T_world = x_posevel.pose;
    const Eigen::Vector3d&   p_world = x_point;

    // Transform landmark to camera/robot frame
    Eigen::Vector3d p_cam = T_world.inverse() * p_world;

    // Build extended 4x4 stereo projection matrix:
    //   Rows 0-2: standard left-camera projection  [P_]
    //   Row 3:    right-camera x-coordinate row    [fx 0 cx -fx*b]
    Eigen::Matrix<double, 4, 4> P_ext = Eigen::Matrix<double, 4, 4>::Zero();
    P_ext.block<3, 4>(0, 0) = P_;
    P_ext.block<1, 4>(3, 0) = P_.block<1, 4>(0, 0);
    P_ext(3, 3)              = -P_(0, 0) * baseline_;

    // Project: [xl; y; z_cam; xr] = P_ext * [p_cam; 1]
    Eigen::Vector4d proj = P_ext.block<4, 3>(0, 0) * p_cam + P_ext.block<4, 1>(0, 3);

    const double xl = proj(0);
    const double y  = proj(1);
    const double z  = proj(2);
    const double xr = proj(3);

    if (z <= 0) {
      inlier_ = false;
      error_  = Eigen::Vector3d::Zero();
      J_      = MatrixJacobianDim::Zero();
      return;
    }

    const double iz  = 1.0 / z;
    const double iz2 = iz * iz;

    Eigen::Vector3d h;
    h(0) = xl * iz;           // u_left
    h(1) = y * iz;            // v
    h(2) = (xl - xr) * iz;   // disparity

    if (!insideImage(h)) {
      inlier_ = false;
      error_  = Eigen::Vector3d::Zero();
      J_      = MatrixJacobianDim::Zero();
      return;
    }

    inlier_ = true;
    error_  = z_ - h;

    // Jacobian of h = [u_left, v, disp] w.r.t. [xl, y, z_cam, xr]
    Eigen::Matrix<double, 3, 4> J_hom;
    J_hom << iz,  0,   -xl * iz2,        0,
              0,  iz,   -y * iz2,        0,
              iz,  0,  (xr - xl) * iz2, -iz;

    // Jacobian of [xl; y; z_cam; xr] w.r.t. SE3PoseVel (12-dim: [t(3), r(3), v(6)])
    //   Translation part: d(p_cam)/d(delta_t) = -I (right perturbation)
    //   Rotation part:    d(p_cam)/d(delta_r) = skew(p_cam) (right perturbation)
    //   Velocity part:    zero (velocity does not affect stereo projection)
    Eigen::Matrix<double, 4, 12> J_proj = Eigen::Matrix<double, 4, 12>::Zero();
    J_proj.block<4, 3>(0, 0) = -P_ext.block<4, 3>(0, 0);
    J_proj.block<4, 3>(0, 3) =  2.0 * P_ext.block<4, 3>(0, 0) * skew(p_cam);

    // Jacobian of [xl; y; z_cam; xr] w.r.t. SE3Point (world frame)
    Eigen::Matrix<double, 4, 3> J_lm = P_ext.block<4, 3>(0, 0) * T_world.linear().transpose();

    // error = z_ - h, so J_error = -J_h (sign flip relative to measurement Jacobian)
    MatrixJacobianDim J = MatrixJacobianDim::Zero();
    J.block<3, 12>(0, 0) = J_hom * J_proj;
    J.block<3, 3>(0, 12)  = J_hom * J_lm;
    J_                    = -J;
  }

} // namespace gsolver
