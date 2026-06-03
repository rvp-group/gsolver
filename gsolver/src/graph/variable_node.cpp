#include <gsolver/graph/variable_node.h>

namespace gsolver {

  template <int Dim_, typename EstimateType_, typename EdgeType_>
  VariableNode<Dim_, EstimateType_, EdgeType_>::VariableNode(const std::string& id,
                                                             const EstimateType_& mu,
                                                             const MatrixLieDim& Sigma) {
    id_    = id;
    mu_    = mu;
    Sigma_ = Sigma;
  }

  template <int Dim_, typename EstimateType_, typename EdgeType_>
  void VariableNode<Dim_, EstimateType_, EdgeType_>::sendMessage(size_t outedge_id) {
    VectorLieDim tangent_eta         = VectorLieDim::Zero();
    MatrixLieDim tangent_Lambdaprime = MatrixLieDim::Zero();
    VectorLieDim tangent_delta_mu    = VectorLieDim::Zero();

    // If there are no factors connected to the variable node, skip the message computation
    if (edges_.size() == 0) {
      return;
    }

    // If there are not enough factors skip the message computation
    if (edges_.size() == 1) {
      return;
    }

    // Multiply inward messages from other factors
    for (size_t i = 0; i < edges_.size(); ++i) {
      if (i == outedge_id) {
        // Skip the outgoing edge
        continue;
      }

      // Access messages from edges (TODO: replace with appropriate getter methods based on your edge structure)
      const MatrixLieDim& Lambdaprime_inward = edges_[i]->factor_to_variable_message_.Lambda_;
      const EstimateType_ mu_inward          = edges_[i]->factor_to_variable_message_.mu_;

      // Project to the tangent space
      VectorLieDim tangent_delta_mu_inward = boxminus(mu_inward);

      // Compute the Lambda and eta
      MatrixLieDim tangent_Lambda_inward = Lambdaprime_inward;
      VectorLieDim tangent_eta_inward    = tangent_Lambda_inward * tangent_delta_mu_inward;

      tangent_eta += tangent_eta_inward;
      tangent_Lambdaprime += tangent_Lambda_inward;
    }
    // Ensure that the tangent_Lambdaprime is invertible
    tangent_Lambdaprime += MatrixLieDim::Identity() * 1e-6; // Add a small perturbation to make it invertible
    // get the perturbation to be applied to the current estimate
    tangent_delta_mu = tangent_Lambdaprime.inverse() * tangent_eta;

    // compute the new estimate in the manifold
    EstimateType_ mu_outward         = boxplus(tangent_delta_mu);
    MatrixLieDim Lambdaprime_outward = tangent_Lambdaprime;

    edges_[outedge_id]->variable_to_factor_message_.Lambda_ = Lambdaprime_outward;
    edges_[outedge_id]->variable_to_factor_message_.mu_     = mu_outward;
  }

  template <int Dim_, typename EstimateType_, typename EdgeType_>
  void VariableNode<Dim_, EstimateType_, EdgeType_>::sendMessage(const std::string& factor_id) {
    VectorLieDim tangent_eta         = VectorLieDim::Zero();
    MatrixLieDim tangent_Lambdaprime = MatrixLieDim::Zero();
    VectorLieDim tangent_delta_mu    = VectorLieDim::Zero();

    // If there are no factors connected to the variable node, skip the message computation
    if (edges_.size() == 0) {
      return;
    }

    // If there are not enough factors skip the message computation
    if (edges_.size() == 1) {
      return;
    }

    // Find the outgoing edge index
    const auto it = std::find_if(edges_.begin(), edges_.end(), [&](const auto& edge) {
      return edge->factor_id_ == factor_id;
    });
    if (it == edges_.end()) {
      throw std::runtime_error("Edge with the given factor_id not found in VariableNode::sendMessage");
    }
    const size_t outedge_id = static_cast<size_t>(std::distance(edges_.begin(), it));

    // Multiply inward messages from other factors
    for (size_t i = 0; i < edges_.size(); ++i) {
      if (i == outedge_id) {
        // Skip the outgoing edge
        continue;
      }

      // Access messages from edges (TODO: replace with appropriate getter methods based on your edge structure)
      const MatrixLieDim& Lambdaprime_inward = edges_[i]->factor_to_variable_message_.Lambda_;
      const EstimateType_ mu_inward          = edges_[i]->factor_to_variable_message_.mu_;

      // Project to the tangent space
      VectorLieDim tangent_delta_mu_inward = boxminus(mu_inward);

      // Compute the Lambda and eta
      MatrixLieDim tangent_Lambda_inward = Lambdaprime_inward;
      VectorLieDim tangent_eta_inward    = tangent_Lambda_inward * tangent_delta_mu_inward;

      tangent_eta += tangent_eta_inward;
      tangent_Lambdaprime += tangent_Lambda_inward;
    }
    // Ensure that the tangent_Lambdaprime is invertible
    tangent_Lambdaprime += MatrixLieDim::Identity() * 1e-6; // Add a small perturbation to make it invertible
    // get the perturbation to be applied to the current estimate
    tangent_delta_mu = tangent_Lambdaprime.inverse() * tangent_eta;

    // compute the new estimate in the manifold
    EstimateType_ mu_outward         = boxplus(tangent_delta_mu);
    MatrixLieDim Lambdaprime_outward = tangent_Lambdaprime;

    edges_[outedge_id]->variable_to_factor_message_.Lambda_ = Lambdaprime_outward;
    edges_[outedge_id]->variable_to_factor_message_.mu_     = mu_outward;
  }

  template <int Dim_, typename EstimateType_, typename EdgeType_>
  void VariableNode<Dim_, EstimateType_, EdgeType_>::sendAllMessages() {
#pragma omp parallel for
    for (size_t i = 0; i < edges_.size(); ++i) {
      sendMessage(i);
    }
  }

  template <int Dim_, typename EstimateType_, typename EdgeType_>
  void VariableNode<Dim_, EstimateType_, EdgeType_>::updateMuAndSigma() {
    VectorLieDim tangent_eta         = VectorLieDim::Zero();
    MatrixLieDim tangent_Lambdaprime = MatrixLieDim::Zero();
    VectorLieDim tangent_delta_mu    = VectorLieDim::Zero();

    // If there are no factors connected to the variable node, skip the update
    if (edges_.size() == 0) {
      return;
    }

    // Multiply inward messages from other factors
    for (size_t i = 0; i < edges_.size(); ++i) {
      // Access messages from edges (replace with appropriate getter methods based on your edge structure)
      const MatrixLieDim& Lambdaprime_inward = edges_[i]->factor_to_variable_message_.Lambda_;
      const EstimateType_ mu_inward          = edges_[i]->factor_to_variable_message_.mu_;

      // Handle the pose
      VectorLieDim tangent_delta_mu_inward = boxminus(mu_inward);

      // Compute the Lambda and eta
      MatrixLieDim tangent_Lambda_inward = Lambdaprime_inward;
      VectorLieDim tangent_eta_inward    = tangent_Lambda_inward * tangent_delta_mu_inward;

      tangent_eta += tangent_eta_inward;
      tangent_Lambdaprime += tangent_Lambda_inward;
    }
    // Ensure that the tangent_Lambdaprime is invertible
    tangent_Lambdaprime += MatrixLieDim::Identity() * 1e-6; // Add a small perturbation to make it invertible
    // get the perturbation to be applied to the current estimate
    tangent_delta_mu = tangent_Lambdaprime.inverse() * tangent_eta;

    // compute and update the estimate in the manifold
    mu_                      = boxplus(tangent_delta_mu);
    MatrixLieDim Lambdaprime = tangent_Lambdaprime;
    Sigma_                   = Lambdaprime.inverse();
  }

} // namespace gsolver