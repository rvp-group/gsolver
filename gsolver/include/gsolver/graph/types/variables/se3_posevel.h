#pragma once

#include <Eigen/Dense>

#include <gsolver/graph/variable_node.h>
#include <gsolver/maths/geometry3d.h>

namespace gsolver {

  struct SE3PoseVelValue {
    Eigen::Isometry3d pose;
    Eigen::Vector<double, 6> vel;
  };

  class SE3PoseVel : public VariableNode<12, SE3PoseVelValue, Edge<SE3PoseVel>> {
  public:
    using EstimateType = SE3PoseVelValue;
    double timestamp_; // Timestamp associated with the pose

    SE3PoseVel(const std::string& id, const EstimateType& mu, const MatrixLieDim& Sigma, double timestamp);

    const VectorLieDim boxminus(const EstimateType& mu_inward) override;

    const EstimateType boxplus(const VectorLieDim& tau) override;

    static const EstimateType boxplus(const EstimateType& mu, const VectorLieDim& tau);
  };

} // namespace gsolver
