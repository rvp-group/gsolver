#pragma once

#include <Eigen/Dense>

#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/types/variables/se3_pose.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/derivatives.h>
#include <gsolver/maths/geometry3d.h>

namespace gsolver {

  class SE3PoseVelPose : public FactorNode<6, SE3PoseVel, SE3Pose> {
  public:
    Eigen::Isometry3d z_;

    SE3PoseVelPose(std::string id, const Eigen::Isometry3d& z, MatrixErrorDim Sigma);

    void updateErrorAndJacobian() override;
  };

} // namespace gsolver