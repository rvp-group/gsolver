#pragma once

#include <gsolver/graph/edge.h>
#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/variable_node.h>

namespace gsolver {

  class FactorGraph {
  public:
    std::vector<std::shared_ptr<VariableNodeBase>> variable_nodes_;
    std::vector<std::shared_ptr<FactorNodeBase>> factor_nodes_;

    template <typename VariableType>
    void addVariableNode(const std::shared_ptr<VariableType>& variable_node);

    template <typename FactorType>
    void addFactorNode(const std::shared_ptr<FactorType>& factor_node, std::vector<std::string> variable_ids);

    // Overload that accepts variable nodes directly — needed when variables live
    // in different factor graphs (e.g. inter-robot landmark equality factors).
    template <typename FactorType>
    void addFactorNode(const std::shared_ptr<FactorType>& factor_node,
                       std::vector<std::shared_ptr<VariableNodeBase>> variable_nodes);

    std::shared_ptr<VariableNodeBase> getVariableNode(const std::string& id);

    std::shared_ptr<FactorNodeBase> getFactorNode(const std::string& id);

    // returns the pose variable nodes sorted by timestamp
    std::vector<std::shared_ptr<VariableNodeBase>> getPoseVariableNodes(); // TODO: revisit the method
  };

} // namespace gsolver
