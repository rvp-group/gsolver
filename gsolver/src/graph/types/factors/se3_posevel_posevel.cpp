#include <gsolver/graph/types/factors/se3_posevel_posevel.h>

namespace gsolver {

  SE3PoseVelPoseVel::SE3PoseVelPoseVel(std::string id, const Eigen::Isometry3d& z, MatrixErrorDim Sigma) : FactorNode(id, Sigma) {
    z_ = z;
  }

  void SE3PoseVelPoseVel::updateErrorAndJacobian() {
    // Get the variables from the edges
    std::shared_ptr<Edge<SE3PoseVel>> edge_from = edges_.at<0>();
    std::shared_ptr<Edge<SE3PoseVel>> edge_to   = edges_.at<1>();
    Message<SE3PoseVel> message_from            = edge_from->variable_to_factor_message_;
    Message<SE3PoseVel> message_to              = edge_to->variable_to_factor_message_;
    SE3PoseVel::EstimateType x_from             = message_from.mu_;
    SE3PoseVel::EstimateType x_to               = message_to.mu_;
    const Eigen::Isometry3d& x_from_pose        = x_from.pose;
    const Eigen::Isometry3d& x_to_pose          = x_to.pose;

    // Compute the observation
    const Eigen::Isometry3d& h_T = x_from_pose.inverse() * x_to_pose;

    // Compute error
    Eigen::Isometry3d error_T = z_.inverse() * h_T;
    error_                    = logMapSE3(error_T);

    // Compute the Jacobian
    J_ = DerivativeUtils::compute_se3vel_delta_jacobian(x_from_pose, x_to_pose, z_);
  }

} // namespace gsolver
