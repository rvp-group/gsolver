#include <gsolver/graph/types/factors/se3_posevel_prior.h>

namespace gsolver {

  SE3PoseVelPrior::SE3PoseVelPrior(std::string id, const Eigen::Isometry3d& z, MatrixErrorDim Sigma) : FactorNode(id, Sigma) {
    z_ = z;
  }

  void SE3PoseVelPrior::updateErrorAndJacobian() {
    // Get the variables from the edges
    std::shared_ptr<Edge<SE3PoseVel>> edge_from = edges_.at<0>();
    Message<SE3PoseVel> message_from            = edge_from->variable_to_factor_message_;
    SE3PoseVel::EstimateType x_prior            = message_from.mu_;
    const Eigen::Isometry3d& x                  = x_prior.pose;

    // Compute the error
    Eigen::Isometry3d error_T = z_.inverse() * x;
    error_                    = logMapSE3(error_T);

    // Compute the Jacobian
    J_ = DerivativeUtils::compute_se3vel_prior_jacobian(x, z_);
  }

} // namespace gsolver
