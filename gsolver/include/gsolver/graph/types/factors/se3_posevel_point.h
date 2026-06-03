#pragma once

#include <Eigen/Dense>

#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/types/variables/se3_point.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/geometry3d.h>

namespace gsolver {

  class SE3PoseVelPoint : public FactorNode<3, SE3PoseVel, SE3Point> {
  public:
    Eigen::Vector3d z_;

    SE3PoseVelPoint(std::string id, const Eigen::Vector3d& z, MatrixErrorDim Sigma);

    void updateErrorAndJacobian() override;
  };

} // namespace gsolver