#pragma once

#include <Eigen/Dense>

#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/types/variables/se3_point.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/maths/geometry3d.h>

namespace gsolver {

  /**
   * @brief Stereo camera factor between an SE3PoseVel variable and an SE3Point landmark.
   *
   * The measurement model projects a 3D landmark into stereo pixel coordinates:
   *   h = [u_left, v, disparity]  where  disparity = u_left - u_right
   *
   * The Jacobian follows the right-perturbation convention used throughout gsolver.
   */
  class SE3PoseVelStereoPoint : public FactorNode<3, SE3PoseVel, SE3Point> {
  public:
    Eigen::Vector3d z_;                 ///< Stereo observation [u_left, v, disparity]
    Eigen::Matrix<double, 3, 4> P_;    ///< Camera projection matrix [fx 0 cx 0; 0 fy cy 0; 0 0 1 0]
    double baseline_;                  ///< Camera baseline (metres)
    bool inlier_ = true;               ///< False when the point is behind the camera or outside image

    SE3PoseVelStereoPoint(std::string id,
                          const Eigen::Vector3d& z,
                          const Eigen::Matrix<double, 3, 4>& P,
                          double baseline,
                          MatrixErrorDim Sigma);

    void updateErrorAndJacobian() override;

  private:
    inline bool insideImage(const Eigen::Vector3d& h) const {
      return h(2) > 0 && (h(0) - h(2)) >= 0;
    }
  };

} // namespace gsolver
