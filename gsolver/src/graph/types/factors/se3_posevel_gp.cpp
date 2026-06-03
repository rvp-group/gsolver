#include <gsolver/graph/types/factors/se3_posevel_gp.h>

namespace gsolver {

  // implement the constructor
  SE3PoseVelGP::SE3PoseVelGP(std::string id, const double dt, MatrixErrorDim Sigma) : FactorNode(id, Sigma) {
    dt_ = dt;
  }

  void SE3PoseVelGP::updateErrorAndJacobian() {
    // Get the variables from the edges
    std::shared_ptr<Edge<SE3PoseVel>> edge_from = edges_.at<0>();
    std::shared_ptr<Edge<SE3PoseVel>> edge_to   = edges_.at<1>();
    Message<SE3PoseVel> message_from            = edge_from->variable_to_factor_message_;
    Message<SE3PoseVel> message_to              = edge_to->variable_to_factor_message_;
    SE3PoseVel::EstimateType x_from             = message_from.mu_;
    SE3PoseVel::EstimateType x_to               = message_to.mu_;
    const Eigen::Isometry3d& x_from_pose        = x_from.pose;
    const Eigen::Isometry3d& x_to_pose          = x_to.pose;
    const Eigen::Vector<double, 6>& x_from_vel  = x_from.vel;
    const Eigen::Vector<double, 6>& x_to_vel    = x_to.vel;

    // Compute error
    error_                           = Eigen::Vector<double, 12>::Zero();
    Eigen::Vector<double, 6> error_p = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 6> error_v = Eigen::Vector<double, 6>::Zero();
    error_p                          = logMapSE3(expMapSE3(dt_ * x_from_vel).inverse() * x_from_pose.inverse() * x_to_pose);
    Eigen::Matrix3d R                = x_from_pose.linear().transpose() * x_to_pose.linear();
    Eigen::Matrix<double, 6, 6> adj  = Eigen::Matrix<double, 6, 6>::Zero();
    adj.block(0, 0, 3, 3)            = R;
    adj.block(3, 3, 3, 3)            = R;
    error_v                          = (adj * x_to_vel) - x_from_vel;
    error_.head(6)                   = error_p;
    error_.tail(6)                   = error_v;

    // Compute the Jacobian
    J_ = DerivativeUtils::compute_se3vel_gp_jacobian(x_from_pose, x_to_pose, x_from_vel, x_to_vel, dt_);
  }

} // namespace gsolver
