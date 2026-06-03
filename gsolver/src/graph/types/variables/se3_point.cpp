#include <gsolver/graph/types/variables/se3_point.h>

namespace gsolver {

  // Type aliases from base class
  using EstimateType = SE3Point::EstimateType;
  using VectorLieDim = SE3Point::VectorLieDim;
  using MatrixLieDim = SE3Point::MatrixLieDim;

  SE3Point::SE3Point(const std::string& id, const EstimateType& mu, const MatrixLieDim& Sigma)
      : VariableNode(id, mu, Sigma) {}

  const VectorLieDim SE3Point::boxminus(const EstimateType& mu_inward) {
    return mu_inward - mu_;
  }

  const EstimateType SE3Point::boxplus(const VectorLieDim& tau_inward) {
    return EstimateType(static_cast<Eigen::Vector3d>(mu_) + tau_inward);
  }

  const EstimateType SE3Point::boxplus(const EstimateType& mu, const VectorLieDim& tau_inward) {
    return EstimateType(static_cast<Eigen::Vector3d>(mu) + tau_inward);
  }

} // namespace gsolver
