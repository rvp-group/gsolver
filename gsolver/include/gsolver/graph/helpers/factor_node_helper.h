#pragma once

#include <Eigen/Dense>

namespace gsolver {

  // compute the marginalization using the Shur complement
  template <int a_size, int b_size>
  static void marginalize(Eigen::Vector<double, a_size + b_size>& eta,
                          Eigen::Matrix<double, a_size + b_size, a_size + b_size>& Lambdaprime,
                          Eigen::Vector<double, a_size>& eta_marg,
                          Eigen::Matrix<double, a_size, a_size>& Lambdaprime_marg) {
    // Split eta into blocks a and b
    Eigen::Vector<double, a_size> ea = eta.head(a_size);
    Eigen::Vector<double, b_size> eb = eta.tail(b_size);
    // Split Lambdaprime into blocks aa, ab, ba, bb
    Eigen::Matrix<double, a_size, a_size> aa = Lambdaprime.block(0, 0, a_size, a_size);
    Eigen::Matrix<double, a_size, b_size> ab = Lambdaprime.block(0, a_size, a_size, b_size);
    Eigen::Matrix<double, b_size, a_size> ba = Lambdaprime.block(a_size, 0, b_size, a_size);
    Eigen::Matrix<double, b_size, b_size> bb = Lambdaprime.block(a_size, a_size, b_size, b_size);
    // Ensure that bb is invertible
    bb += Eigen::Matrix<double, b_size, b_size>::Identity() * 1e-6;

    // Marginalize
    eta_marg         = ea - ab * bb.inverse() * eb;
    Lambdaprime_marg = aa - ab * bb.inverse() * ba;
  }

  // compute the message from the factor node to the variable node
  template <typename VariableType, int a_offset, int a_size, int b_size>
  static Message<VariableType> computeMessage(Eigen::Vector<double, a_size + b_size>& factor_eta,
                                              Eigen::Matrix<double, a_size + b_size, a_size + b_size>& factor_Lambdaprime,
                                              Eigen::Matrix<double, a_size + b_size, a_size + b_size>& conditioned_Lambdaprime,
                                              typename VariableType::EstimateType& mu_to_update) {
    // Define the types
    using EstimateType          = typename VariableType::EstimateType;
    static const int total_size = a_size + b_size; // Total size of the joint distribution

    // Compute the marginalization for the outgoing edge
    Eigen::Vector<double, a_size> eta_M;
    Eigen::Matrix<double, a_size, a_size> Lambdaprime_M;
    if constexpr (a_size == total_size) {
      // If the output size is equal to the total dimension, no marginalization is needed (already a marginal)
      eta_M         = factor_eta.head(a_size);
      Lambdaprime_M = factor_Lambdaprime.block(a_offset, a_offset, a_size, a_size);
    } else {
      // Rearrange the eta and Lambdaprime to have outward information on top and the conditioned information on the bottom
      // Rearrange eta
      Eigen::Vector<double, total_size> eta_C = factor_eta;
      eta_C.segment(0, a_size)                = factor_eta.segment(a_offset, a_size);
      eta_C.segment(a_offset, a_size)         = factor_eta.segment(0, a_size);
      // Rearrange Lambdaprime
      Eigen::Matrix<double, total_size, total_size> Lambdaprime_C = conditioned_Lambdaprime;
      Lambdaprime_C.block(a_offset, a_offset, a_size, a_size)     = factor_Lambdaprime.block(a_offset, a_offset, a_size, a_size);
      // Rearrange Lambdaprime rows
      Eigen::Matrix<double, a_size, total_size> row_temp   = Lambdaprime_C.block(0, 0, a_size, total_size);
      Lambdaprime_C.block(0, 0, a_size, total_size)        = Lambdaprime_C.block(a_offset, 0, a_size, total_size);
      Lambdaprime_C.block(a_offset, 0, a_size, total_size) = row_temp;
      // Rearrange Lambdaprime columns
      Eigen::Matrix<double, total_size, a_size> col_temp   = Lambdaprime_C.block(0, 0, total_size, a_size);
      Lambdaprime_C.block(0, 0, total_size, a_size)        = Lambdaprime_C.block(0, a_offset, total_size, a_size);
      Lambdaprime_C.block(0, a_offset, total_size, a_size) = col_temp;

      // compute the marginalization using the Shur complement
      marginalize<a_size, b_size>(eta_C, Lambdaprime_C, eta_M, Lambdaprime_M);
    }

    // ensure that the Lambdaprime_marg is invertible
    Lambdaprime_M += Eigen::Matrix<double, a_size, a_size>::Identity() * 1e-6;

    // Update the estimate to send
    Eigen::Vector<double, a_size> tau                    = Lambdaprime_M.inverse() * eta_M;
    EstimateType mu_outward                              = VariableType::boxplus(mu_to_update, tau);
    Eigen::Matrix<double, a_size, a_size> Lambda_outward = Lambdaprime_M;

    // Return the message
    Message<VariableType> message;
    message.Lambda_ = Lambda_outward;
    message.mu_     = mu_outward;
    return message;
  }

} // namespace gsolver