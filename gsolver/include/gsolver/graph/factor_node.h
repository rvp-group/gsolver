#pragma once

#include <Eigen/Dense>

#include <gsolver/core/robust_kernels.h>
#include <gsolver/graph/edge.h>
#include <gsolver/graph/factor_node_base.h>
#include <gsolver/graph/helpers/factor_node_edge_tuple_helper.h>

namespace gsolver {

  template <int Dim_, typename... VariableTypes_>
  class FactorNode : public FactorNodeBase {
  public:
    static constexpr int VariablesDim_  = (0 + ... + VariableTypes_::LieDim);
    static constexpr int NumberOfEdges_ = sizeof...(VariableTypes_);

    using VariableTypes      = std::tuple<VariableTypes_...>;
    using MatrixErrorDim     = Eigen::Matrix<double, Dim_, Dim_>;
    using MatrixVariablesDim = Eigen::Matrix<double, VariablesDim_, VariablesDim_>;
    using VectorErrorDim     = Eigen::Vector<double, Dim_>;
    using VectorVariablesDim = Eigen::Vector<double, VariablesDim_>;
    using MatrixJacobianDim  = Eigen::Matrix<double, Dim_, VariablesDim_>;
    using EdgeTuple          = FactorEdgeTuple_<VariableTypes_...>;

    FactorNode(const std::string& id, const MatrixErrorDim& Sigma);

    // Estimate of the factor (canonical form)
    VectorVariablesDim eta_;         // eta vector (information vector of the factor)
    MatrixVariablesDim Lambdaprime_; // Lambda matrix (precision matrix of the factor)

    // Conditioned estimate (used for message computation)
    MatrixVariablesDim Lambdaprime_conditioned_; // Lambda matrix conditioned on all the connected edges

    // error and Jacobian of the factor
    VectorErrorDim error_; // Error of the factor
    MatrixErrorDim Sigma_; // Factor covariance matrix (represents uncertainty of the error)
    MatrixJacobianDim J_;  // Jacobian of the factor

    // Edges connected to this factor node
    EdgeTuple edges_; // Edges connected to this factor node

    void setEdgeAt(size_t i, std::shared_ptr<EdgeBase> edge) override {
      edges_.setEdgeAt(i, edge);
    }

    // Update the factor based on the connected edges
    void updateFactor() override;

    // Send message from factor node to variable node
    void sendMessage(const std::string& variable_id) override;

    // Send messages to all connected variable nodes
    void sendAllMessages() override;

    // Update error and Jacobian of the factor
    // This function should be implemented in the derived class
    virtual void updateErrorAndJacobian() = 0;
  };

} // namespace gsolver
