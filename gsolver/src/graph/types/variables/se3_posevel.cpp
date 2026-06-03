#include <gsolver/graph/types/variables/se3_posevel.h>

namespace gsolver {

  using EstimateType = SE3PoseVel::EstimateType;
  using VectorLieDim = SE3PoseVel::VectorLieDim;
  using MatrixLieDim = SE3PoseVel::MatrixLieDim;

  SE3PoseVel::SE3PoseVel(const std::string& id, const EstimateType& mu, const MatrixLieDim& Sigma, double timestamp) :
    VariableNode(id, mu, Sigma), timestamp_(timestamp) {
  }

  const VectorLieDim SE3PoseVel::boxminus(const EstimateType& mu_inward) {
    VectorLieDim tau;
    tau.head(6) = logMapSE3(mu_.pose.inverse() * mu_inward.pose);
    tau.tail(6) = mu_inward.vel - mu_.vel;
    return tau;
  }

  const EstimateType SE3PoseVel::boxplus(const VectorLieDim& tau) {
    EstimateType mu_outward;
    mu_outward.pose = mu_.pose * expMapSE3(tau.head(6));
    mu_outward.vel  = mu_.vel + tau.tail(6);
    return mu_outward;
  }

  const EstimateType SE3PoseVel::boxplus(const EstimateType& mu, const VectorLieDim& tau) {
    EstimateType mu_outward;
    mu_outward.pose = mu.pose * expMapSE3(tau.head(6));
    mu_outward.vel  = mu.vel + tau.tail(6);
    return mu_outward;
  }

} // namespace gsolver
