#include <gsolver/graph/types/factors/se3_posevel_point.h>

namespace gsolver {

  SE3PoseVelPoint::SE3PoseVelPoint(std::string id, const Eigen::Vector3d& z, MatrixErrorDim Sigma) : FactorNode(id, Sigma) {
    z_ = z;
  }

  void SE3PoseVelPoint::updateErrorAndJacobian() {
    // Get the variables from the edges
    std::shared_ptr<Edge<SE3PoseVel>> edge_from = edges_.at<0>();
    std::shared_ptr<Edge<SE3Point>> edge_to     = edges_.at<1>();
    Message<SE3PoseVel> message_from            = edge_from->variable_to_factor_message_;
    Message<SE3Point> message_to                = edge_to->variable_to_factor_message_;
    const SE3PoseVel::EstimateType& x0_se3vel   = message_from.mu_;
    const SE3Point::EstimateType& x0_point3d    = message_to.mu_;

    // Compute the rotation matrix
    Eigen::Matrix<double, 3, 3> R     = x0_se3vel.pose.linear();
    Eigen::Matrix<double, 3, 3> R_inv = R.inverse();

    // Compute the observation
    Eigen::Vector3d h = R_inv * (x0_point3d - x0_se3vel.pose.translation());

    // Compute error
    error_ = z_ - h;

    // Compute the Jacobian
    MatrixJacobianDim J  = MatrixJacobianDim::Zero();
    J.block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    J.block<3, 3>(0, 3)  = -skew(h);
    J.block<3, 3>(0, 12) = -R_inv;
    J_                   = J;
  }

} // namespace gsolver