#pragma once

#include <Eigen/Dense>

#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/helpers/factor_node_edge_tuple_helper.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/derivatives.h>
#include <gsolver/maths/geometry3d.h>

namespace gsolver {

  class SE3PoseVelGP : public FactorNode<12, SE3PoseVel, SE3PoseVel> {
  public:
    double dt_;

    SE3PoseVelGP(std::string id, const double dt, MatrixErrorDim Sigma);

    void updateErrorAndJacobian() override;
  };

} // namespace gsolver
