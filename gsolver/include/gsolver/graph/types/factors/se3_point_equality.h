#pragma once

#include <Eigen/Dense>

#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/types/variables/se3_point.h>

namespace gsolver {

  /**
   * @brief Equality factor between two SE3Point (3D landmark) variables.
   *
   * Enforces that two landmark estimates refer to the same point in world space.
   * Used in multi-robot SLAM to fuse landmarks observed by different robots.
   *
   * error = x1 - x0
   * J     = [-I | I]
   */
  class SE3PointEquality : public FactorNode<3, SE3Point, SE3Point> {
  public:
    SE3PointEquality(std::string id, MatrixErrorDim Sigma);

    void updateErrorAndJacobian() override;
  };

} // namespace gsolver
