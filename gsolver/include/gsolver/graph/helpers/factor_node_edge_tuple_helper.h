#pragma once

#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

#include <gsolver/graph/edge.h>
#include <gsolver/graph/edge_base.h>
#include <gsolver/graph/helpers/factor_node_helper.h>

namespace gsolver {

  /*! @brief Recursive class to access type of variable i in the tuple */
  template <typename FactorEdgeTuple_, int i>
  struct VariableTypeAt_ {
    using VariableType = typename VariableTypeAt_<typename FactorEdgeTuple_::RestFactorEdgeTuple, i - 1>::VariableType;
  };
  /*! @brief Base case of the recursion for VariableTypeAt_ */
  template <typename FactorEdgeTuple_>
  struct VariableTypeAt_<FactorEdgeTuple_, 0> {
    using VariableType = typename FactorEdgeTuple_::CurrentVariableType;
  };

  // Helper class for variadic template recursion
  template <typename CurrentVariableType_, typename... RestVariableTypes_>
  class FactorEdgeTuple_ : public FactorEdgeTuple_<RestVariableTypes_...> {
  public:
    using ThisType            = FactorEdgeTuple_<CurrentVariableType_, RestVariableTypes_...>;
    using CurrentVariableType = CurrentVariableType_;
    using RestFactorEdgeTuple = FactorEdgeTuple_<RestVariableTypes_...>;

    std::shared_ptr<Edge<CurrentVariableType>> current_edge_;

    template <int i>
    using VariableTypeAt = typename VariableTypeAt_<ThisType, i>::VariableType;

    // Access the variable at index i in the tuple
    template <int i>
    std::shared_ptr<Edge<VariableTypeAt<i>>>& at() {
      if constexpr (i == 0) {
        return current_edge_;
      } else {
        return RestFactorEdgeTuple::template at<i - 1>();
      }
    }

    // Access the edge at index i in the tuple
    std::shared_ptr<EdgeBase> getEdgeAt(size_t i) {
      if (i == 0) {
        return current_edge_;
      }
      return RestFactorEdgeTuple::getEdgeAt(i - 1);
    }

    // Set the edge at index i in the tuple
    void setEdgeAt(size_t i, std::shared_ptr<EdgeBase> edge) {
      if (i == 0) {
        current_edge_ = std::reinterpret_pointer_cast<Edge<CurrentVariableType>>(edge);
        return;
      }
      RestFactorEdgeTuple::setEdgeAt(i - 1, edge);
    }

    // Compute the joint distribution conditioned on the current messages
    template <int VariablesDim_>
    Eigen::Matrix<double, VariablesDim_, VariablesDim_>
    computeJointDistribution(Eigen::Matrix<double, VariablesDim_, VariablesDim_>& Lambdaprime, int offset = 0) {
      static const int edge_size = CurrentVariableType::LieDim;
      Lambdaprime.block(offset, offset, edge_size, edge_size) += current_edge_->variable_to_factor_message_.Lambda_;
      return RestFactorEdgeTuple::computeJointDistribution(Lambdaprime, offset + edge_size);
    }

    // Send message from factor node to variable node
    template <int VariablesDim_, int out_offset_ = 0>
    void sendMessage(std::string variable_id,
                     Eigen::Vector<double, VariablesDim_> factor_eta,
                     Eigen::Matrix<double, VariablesDim_, VariablesDim_> factor_Lambdaprime,
                     Eigen::Matrix<double, VariablesDim_, VariablesDim_> conditioned_Lambdaprime) {
      if (current_edge_->variable_id_ == variable_id) {
        static const int total_size = VariablesDim_;               // Total size of the joint distribution
        static const int a_size     = CurrentVariableType::LieDim; // Size of the outgoing edge
        static const int b_size     = total_size - a_size;         // Size of the remaining edges
        // Send the message to the variable node
        current_edge_->factor_to_variable_message_ = computeMessage<CurrentVariableType, out_offset_, a_size, b_size>(
          factor_eta, factor_Lambdaprime, conditioned_Lambdaprime, current_edge_->variable_to_factor_message_.mu_);
        return;
      }
      static const int next_out_offset = out_offset_ + CurrentVariableType::LieDim;
      RestFactorEdgeTuple::template sendMessage<VariablesDim_, next_out_offset>(
        variable_id, factor_eta, factor_Lambdaprime, conditioned_Lambdaprime);
    }

    // Send messages from factor node to all the connected variable node
    template <int VariablesDim_, int out_offset_ = 0>
    void sendAllMessages(Eigen::Vector<double, VariablesDim_> factor_eta,
                         Eigen::Matrix<double, VariablesDim_, VariablesDim_> factor_Lambdaprime,
                         Eigen::Matrix<double, VariablesDim_, VariablesDim_> conditioned_Lambdaprime) {
      static const int total_size      = VariablesDim_;               // Total size of the joint distribution
      static const int a_size          = CurrentVariableType::LieDim; // Size of the outgoing edge
      static const int b_size          = total_size - a_size;         // Size of the remaining edges
      static const int next_out_offset = out_offset_ + CurrentVariableType::LieDim;
      // Send the message to the variable node
      current_edge_->factor_to_variable_message_ = computeMessage<CurrentVariableType, out_offset_, a_size, b_size>(
        factor_eta, factor_Lambdaprime, conditioned_Lambdaprime, current_edge_->variable_to_factor_message_.mu_);
      // Send messages to the rest of the variable nodes
      RestFactorEdgeTuple::template sendAllMessages<VariablesDim_, next_out_offset>(
        factor_eta, factor_Lambdaprime, conditioned_Lambdaprime);
    }
  };

  // Base case for variadic template recursion
  template <typename CurrentVariableType_>
  class FactorEdgeTuple_<CurrentVariableType_> {
  public:
    using ThisType            = FactorEdgeTuple_<CurrentVariableType_>;
    using CurrentVariableType = CurrentVariableType_;

    std::shared_ptr<Edge<CurrentVariableType>> current_edge_;

    template <int i>
    using VariableTypeAt = typename VariableTypeAt_<ThisType, i>::VariableType;

    // Access the variable at index i in the tuple
    template <int i>
    std::shared_ptr<Edge<VariableTypeAt<i>>>& at() {
      if constexpr (i == 0) {
        return current_edge_;
      } else {
        throw std::runtime_error("index out of bounds");
      }
    }

    // Access the edge at index i in the tuple
    std::shared_ptr<EdgeBase> getEdgeAt(size_t i) {
      if (i == 0) {
        return current_edge_;
      }
      throw std::runtime_error("index out of bounds");
    }

    // Set the edge at index i in the tuple
    void setEdgeAt(size_t i, std::shared_ptr<EdgeBase> edge) {
      if (i == 0) {
        current_edge_ = std::reinterpret_pointer_cast<Edge<CurrentVariableType>>(edge);
        return;
      }
      throw std::runtime_error("index out of bounds");
    }

    // Compute the joint distribution conditioned on the current messages
    template <int VariablesDim_>
    Eigen::Matrix<double, VariablesDim_, VariablesDim_>
    computeJointDistribution(Eigen::Matrix<double, VariablesDim_, VariablesDim_>& Lambdaprime, int offset = 0) {
      static const int edge_size = CurrentVariableType::LieDim;
      Lambdaprime.block(offset, offset, edge_size, edge_size) += current_edge_->variable_to_factor_message_.Lambda_;
      return Lambdaprime;
    }

    // Send message from factor node to variable node
    template <int VariablesDim_, int out_offset_ = 0>
    void sendMessage(std::string variable_id,
                     Eigen::Vector<double, VariablesDim_> factor_eta,
                     Eigen::Matrix<double, VariablesDim_, VariablesDim_> factor_Lambdaprime,
                     Eigen::Matrix<double, VariablesDim_, VariablesDim_> conditioned_Lambdaprime) {
      if (current_edge_->variable_id_ == variable_id) {
        static const int total_size = VariablesDim_;               // Total size of the joint distribution
        static const int a_size     = CurrentVariableType::LieDim; // Size of the outgoing edge
        static const int b_size     = total_size - a_size;         // Size of the remaining edges
        // Send the message to the variable node
        current_edge_->factor_to_variable_message_ = computeMessage<CurrentVariableType, out_offset_, a_size, b_size>(
          factor_eta, factor_Lambdaprime, conditioned_Lambdaprime, current_edge_->variable_to_factor_message_.mu_);
        return;
      }
      throw std::runtime_error("variable id not found");
    }

    // Send messages from factor node to all the connected variable node
    template <int VariablesDim_, int out_offset_ = 0>
    void sendAllMessages(Eigen::Vector<double, VariablesDim_> factor_eta,
                         Eigen::Matrix<double, VariablesDim_, VariablesDim_> factor_Lambdaprime,
                         Eigen::Matrix<double, VariablesDim_, VariablesDim_> conditioned_Lambdaprime) {
      static const int total_size = VariablesDim_;               // Total size of the joint distribution
      static const int a_size     = CurrentVariableType::LieDim; // Size of the outgoing edge
      static const int b_size     = total_size - a_size;         // Size of the remaining edges
      // Send the message to the variable node
      current_edge_->factor_to_variable_message_ = computeMessage<CurrentVariableType, out_offset_, a_size, b_size>(
        factor_eta, factor_Lambdaprime, conditioned_Lambdaprime, current_edge_->variable_to_factor_message_.mu_);
      return;
    }
  };

} // namespace gsolver
