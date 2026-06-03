#pragma once

#include <Eigen/Dense>

#include <gsolver/graph/variable_node.h>
#include <gsolver/maths/geometry3d.h>

namespace gsolver {

  class SE3Pose : public VariableNode<6, Eigen::Isometry3d, Edge<SE3Pose>> {
  public:
    SE3Pose(const std::string& id, const EstimateType& mu, const MatrixLieDim& Sigma);

    const VectorLieDim boxminus(const EstimateType& mu_inward) override;

    const EstimateType boxplus(const VectorLieDim& tau) override;

    static const EstimateType boxplus(const EstimateType& mu, const VectorLieDim& tau);
  };

} // namespace gsolver
