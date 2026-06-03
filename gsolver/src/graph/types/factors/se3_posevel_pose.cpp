#include <gsolver/graph/types/factors/se3_posevel_pose.h>

namespace gsolver {

  SE3PoseVelPose::SE3PoseVelPose(std::string id, const Eigen::Isometry3d& z, MatrixErrorDim Sigma) : FactorNode(id, Sigma) {
    z_ = z;
  }

  void SE3PoseVelPose::updateErrorAndJacobian() {
    // Get the variables from the edges
    std::shared_ptr<Edge<SE3PoseVel>> edge_from = edges_.at<0>();
    std::shared_ptr<Edge<SE3Pose>> edge_to      = edges_.at<1>();
    Message<SE3PoseVel> message_from            = edge_from->variable_to_factor_message_;
    Message<SE3Pose> message_to                 = edge_to->variable_to_factor_message_;
    const SE3PoseVel::EstimateType& x0_se3vel   = message_from.mu_;
    const SE3Pose::EstimateType& x0_se3         = message_to.mu_;

    // Compute the observation
    Eigen::Isometry3d h_T = x0_se3vel.pose.inverse() * x0_se3;

    // Compute error
    Isometry3d error_T = z_.inverse() * h_T;
    error_             = logMapSE3(error_T);

    // Compute the Jacobian
    J_ = DerivativeUtils::compute_se3vel_se3_jacobian(x0_se3vel.pose, x0_se3, z_);
  }

} // namespace gsolver