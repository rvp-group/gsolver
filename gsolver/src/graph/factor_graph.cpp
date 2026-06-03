#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/helpers/factor_graph_helper.h>

namespace gsolver {

  template <typename VariableType>
  void FactorGraph::addVariableNode(const std::shared_ptr<VariableType>& variable_node) {
    // Add the variable node to the factor graph
    variable_nodes_.push_back(variable_node);
  }

  template <typename FactorType>
  void FactorGraph::addFactorNode(const std::shared_ptr<FactorType>& factor_node, std::vector<std::string> variable_ids) {
    // Add the factor node to the factor graph
    factor_nodes_.push_back(factor_node);

    // assert that the number of variable ids matches the number of edges in the factor node
    if (variable_ids.size() != factor_node->NumberOfEdges_) {
      throw std::runtime_error("Number of variable ids does not match the number of edges in the factor node: " +
                               std::to_string(variable_ids.size()) + " != " + std::to_string(factor_node->NumberOfEdges_));
    }

    using FactorGraphHelper = typename FactorGraphHelperBuilder<FactorType, typename FactorType::VariableTypes>::type;
    // Create edges between the factor node and variable nodes
    for (int i = 0; i < factor_node->NumberOfEdges_; ++i) {
      // Get the variable node from the factor graph
      std::shared_ptr<VariableNodeBase> variable_node = getVariableNode(variable_ids[i]);
      // Create edge
      FactorGraphHelper::createEdge(i, factor_node, variable_node);
    }
  }

  std::shared_ptr<VariableNodeBase> FactorGraph::getVariableNode(const std::string& id) {
    for (const auto& variable_node : variable_nodes_) {
      if (variable_node->id_ == id) {
        return variable_node;
      }
    }
    return nullptr;
  }

  std::shared_ptr<FactorNodeBase> FactorGraph::getFactorNode(const std::string& id) {
    for (const auto& factor_node : factor_nodes_) {
      if (factor_node->id_ == id) {
        return factor_node;
      }
    }
    return nullptr;
  }

  // returns the pose variable nodes sorted by timestamp
  std::vector<std::shared_ptr<VariableNodeBase>> FactorGraph::getPoseVariableNodes() {
    std::vector<std::shared_ptr<VariableNodeBase>> pose_variable_nodes;
    for (const auto& variable_node : variable_nodes_) {
      if (variable_node->id_.find("POSE_") != std::string::npos) {
        pose_variable_nodes.push_back(variable_node);
      }
    }
    // variable_node->id_ = "POSE_" + timestamp, sort by timestamp
    std::sort(pose_variable_nodes.begin(),
              pose_variable_nodes.end(),
              [](const std::shared_ptr<VariableNodeBase>& a, const std::shared_ptr<VariableNodeBase>& b) {
                return std::stod(a->id_.substr(5)) < std::stod(b->id_.substr(5));
              });
    return pose_variable_nodes;
  }

} // namespace gsolver