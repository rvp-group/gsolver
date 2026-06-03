#pragma once

#include <Eigen/Dense>
#include <memory>

#include <gsolver/graph/edge.h>
#include <gsolver/graph/variable_node_base.h>

namespace gsolver {

  template <int LieDim_, typename EstimateType_, typename EdgeType_>
  class VariableNode : public VariableNodeBase {
  public:
    // using VariableType = VariableNode<LieDim_, EstimateType_>; // Type for variable node
    using EstimateType = EstimateType_; // Type for mu (estimate in manifold)
    using EdgeType     = EdgeType_;     // Type for edge
    using VectorLieDim = Eigen::Matrix<double, LieDim_, 1>;
    using MatrixLieDim = Eigen::Matrix<double, LieDim_, LieDim_>;

    constexpr static int LieDim = LieDim_;

    VariableNode(const std::string& id, const EstimateType_& mu, const MatrixLieDim& Sigma);

    EstimateType mu_;    // Estimate of the variable
    MatrixLieDim Sigma_; // Covariance matrix of the variable

    std::vector<std::shared_ptr<EdgeType>> edges_; // Edges connected to this factor node

    void sendMessage(size_t outedge_id) override;
    void sendMessage(const std::string& factor_id) override;
    void sendAllMessages() override;
    void updateMuAndSigma() override;

    // For each new VariableNode, implement the following functions:
    virtual const VectorLieDim boxminus(const EstimateType& mu_inward) = 0;
    virtual const EstimateType boxplus(const VectorLieDim& tau)        = 0;
    // static const EstimateType boxplus(const EstimateType& mu, const VectorLieDim& tau); // implemented in the derived class
  };

} // namespace gsolver