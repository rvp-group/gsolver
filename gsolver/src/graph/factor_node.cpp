#include <gsolver/graph/factor_node.h>

namespace gsolver {

  template <int Dim_, typename... VariableTypes_>
  FactorNode<Dim_, VariableTypes_...>::FactorNode(const std::string& id, const MatrixErrorDim& Sigma) {
    id_          = id;
    Sigma_       = Sigma;
    eta_         = VectorVariablesDim::Zero();
    Lambdaprime_ = MatrixVariablesDim::Identity();
    error_       = VectorErrorDim::Zero();
    J_           = MatrixJacobianDim::Zero();
  }

  template <int Dim_, typename... VariableTypes_>
  void FactorNode<Dim_, VariableTypes_...>::updateFactor() {
    // Update the error and Jacobian
    updateErrorAndJacobian();
    // Compute the k factor for robust estimation
    // double k = kRHuber(sqrt(error_.transpose() * Sigma_.inverse() * error_));
    double k = 1;
    // Update factor estimate (canonical form)
    eta_         = J_.transpose() * Sigma_.inverse() * (-error_) * k;
    Lambdaprime_ = J_.transpose() * Sigma_.inverse() * J_ * k;

    // Update Lambda by conditioning on the connected edges
    Lambdaprime_conditioned_ = Lambdaprime_;
    Lambdaprime_conditioned_ = edges_.template computeJointDistribution<VariablesDim_>(Lambdaprime_conditioned_);
  }

  template <int Dim_, typename... VariableTypes_>
  void FactorNode<Dim_, VariableTypes_...>::sendMessage(const std::string& variable_id) {
    // Send message from factor node to variable node
    edges_.template sendMessage<VariablesDim_>(variable_id, eta_, Lambdaprime_, Lambdaprime_conditioned_);
  }

  template <int Dim_, typename... VariableTypes_>
  void FactorNode<Dim_, VariableTypes_...>::sendAllMessages() {
    // iterate over all edges and send messages to all connected variable nodes
    edges_.template sendAllMessages<VariablesDim_>(eta_, Lambdaprime_, Lambdaprime_conditioned_);
  }

} // namespace gsolver