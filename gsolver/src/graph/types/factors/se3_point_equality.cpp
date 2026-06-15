#include <gsolver/graph/types/factors/se3_point_equality.h>

namespace gsolver {

  SE3PointEquality::SE3PointEquality(std::string id, MatrixErrorDim Sigma) : FactorNode(id, Sigma) {}

  void SE3PointEquality::updateErrorAndJacobian() {
    std::shared_ptr<Edge<SE3Point>> edge_0 = edges_.at<0>();
    std::shared_ptr<Edge<SE3Point>> edge_1 = edges_.at<1>();
    const SE3Point::EstimateType& x0       = edge_0->variable_to_factor_message_.mu_;
    const SE3Point::EstimateType& x1       = edge_1->variable_to_factor_message_.mu_;

    error_ = x1 - x0;

    J_                    = MatrixJacobianDim::Zero();
    J_.block<3, 3>(0, 0)  = -Eigen::Matrix3d::Identity();
    J_.block<3, 3>(0, 3)  =  Eigen::Matrix3d::Identity();
  }

} // namespace gsolver
