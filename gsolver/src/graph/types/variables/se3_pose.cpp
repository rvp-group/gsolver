#include <gsolver/graph/types/variables/se3_pose.h>

namespace gsolver {

  using EstimateType = SE3Pose::EstimateType;
  using VectorLieDim = SE3Pose::VectorLieDim;
  using MatrixLieDim = SE3Pose::MatrixLieDim;

  SE3Pose::SE3Pose(const std::string& id, const EstimateType& mu, const MatrixLieDim& Sigma) : VariableNode(id, mu, Sigma) {
  }

  const VectorLieDim SE3Pose::boxminus(const EstimateType& mu_inward) {
    VectorLieDim tau_outward = logMapSE3(mu_.inverse() * mu_inward);
    return tau_outward;
  }

  const EstimateType SE3Pose::boxplus(const VectorLieDim& tau) {
    EstimateType mu_outward = mu_ * expMapSE3(tau);
    return mu_outward;
  }

  const EstimateType SE3Pose::boxplus(const EstimateType& mu, const VectorLieDim& tau) {
    EstimateType mu_outward = mu * expMapSE3(tau);
    return mu_outward;
  }

} // namespace gsolver
