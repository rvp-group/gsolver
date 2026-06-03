#pragma once

#include <gsolver/graph/edge.h>
#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/variable_node.h>
#include <gsolver/graph/variable_node_base.h>

namespace gsolver {

  // Helper class for variadic template recursion
  template <typename FactorType_, typename CurrentVariableType_, typename... RestVariableTypes_>
  class FactorGraphHelper_ : public FactorGraphHelper_<FactorType_, RestVariableTypes_...> {
  public:
    using FactorType            = FactorType_;
    using CurrentVariableType   = CurrentVariableType_;
    using RestFactorGraphHelper = FactorGraphHelper_<FactorType_, RestVariableTypes_...>;

    // Recursively create edges between the factor node and variable nodes
    static void createEdge(int i,
                           std::shared_ptr<FactorType> factor_node,
                           std::shared_ptr<VariableNodeBase> variable_node_base,
                           int recoursion_depth = 0) {
      if (recoursion_depth == i) {
        // Check if the variable node is of the correct type and cast it
        std::shared_ptr<CurrentVariableType> variable_node =
          std::reinterpret_pointer_cast<CurrentVariableType>(variable_node_base);
        if (!variable_node) {
          throw std::runtime_error("Variable node type mismatch");
        }

        // Create a new edge between the factor node and the variable node
        std::shared_ptr<Edge<CurrentVariableType>> edge = std::make_shared<Edge<CurrentVariableType>>();
        edge->variable_id_                              = variable_node->id_;
        edge->factor_id_                                = factor_node->id_;

        // Initialize the variable-to-factor message
        edge->variable_to_factor_message_.mu_     = variable_node->mu_;
        edge->variable_to_factor_message_.Lambda_ = variable_node->Sigma_.inverse();

        // Add the edge to the factor node and variable node
        factor_node->setEdgeAt(i, edge);
        variable_node->edges_.push_back(edge);
        return;
      }
      RestFactorGraphHelper::createEdge(i, factor_node, variable_node_base, recoursion_depth + 1);
    }
  };

  // Base case for variadic template recursion
  template <typename FactorType_, typename CurrentVariableType_>
  class FactorGraphHelper_<FactorType_, CurrentVariableType_> {
  public:
    using FactorType          = FactorType_;
    using CurrentVariableType = CurrentVariableType_;

    // Recursively create edges between the factor node and variable nodes
    static void createEdge(int i,
                           std::shared_ptr<FactorType> factor_node,
                           std::shared_ptr<VariableNodeBase> variable_node_base,
                           int recoursion_depth = 0) {
      if (recoursion_depth == i) {
        // Check if the variable node is of the correct type and cast it
        std::shared_ptr<CurrentVariableType> variable_node =
          std::reinterpret_pointer_cast<CurrentVariableType>(variable_node_base);
        if (!variable_node) {
          throw std::runtime_error("Variable node type mismatch");
        }

        // Create a new edge between the factor node and the variable node
        std::shared_ptr<Edge<CurrentVariableType>> edge = std::make_shared<Edge<CurrentVariableType>>();
        edge->variable_id_                              = variable_node->id_;
        edge->factor_id_                                = factor_node->id_;

        // Initialize the variable-to-factor message
        edge->variable_to_factor_message_.mu_     = variable_node->mu_;
        edge->variable_to_factor_message_.Lambda_ = variable_node->Sigma_.inverse();

        // Add the edge to the factor node and variable node
        factor_node->setEdgeAt(i, edge);
        variable_node->edges_.push_back(edge);
        return;
      }
      throw std::runtime_error("index out of bounds");
    }
  };

  // Helper to unpack tuple
  template <typename FactorNode_, typename Tuple>
  struct FactorGraphHelperBuilder;

  template <typename FactorNode_, typename... VariableNodes_>
  struct FactorGraphHelperBuilder<FactorNode_, std::tuple<VariableNodes_...>> {
    using type = FactorGraphHelper_<FactorNode_, VariableNodes_...>;
  };

} // namespace gsolver