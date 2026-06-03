/**
 * @file instantiations.cpp
 * @brief Explicit template instantiations for all gsolver types.
 *
 * This file contains all explicit template instantiations for VariableNode,
 * FactorNode, and FactorGraph template methods. By centralizing instantiations
 * here, we:
 *   1. Compile template code only once (faster builds)
 *   2. Keep type-specific files clean
 *   3. Make it easy to see all supported types at a glance
 *
 * When adding a new variable or factor type, add the corresponding
 * instantiation here.
 */

// Include template definitions (the .cpp files with template implementations)
#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/factor_node.h>
#include <gsolver/graph/variable_node.h>

// Include all concrete types
#include <gsolver/graph/types/all_types.h>

// Include template implementations
#include "graph/factor_graph.cpp"
#include "graph/factor_node.cpp"
#include "graph/variable_node.cpp"

namespace gsolver {

  // =============================================================================
  // VariableNode Instantiations
  // =============================================================================
  template class VariableNode<12, SE3PoseVelValue, Edge<SE3PoseVel>>; // SE3PoseVel
  template class VariableNode<6, Eigen::Isometry3d, Edge<SE3Pose>>;   // SE3Pose
  template class VariableNode<3, Eigen::Vector3d, Edge<SE3Point>>;    // SE3Point

  // =============================================================================
  // FactorNode Instantiations
  // =============================================================================
  template class FactorNode<6, SE3PoseVel>;              // SE3PoseVelPrior
  template class FactorNode<6, SE3PoseVel, SE3PoseVel>;  // SE3PoseVelPoseVel
  template class FactorNode<12, SE3PoseVel, SE3PoseVel>; // SE3PoseVelGP
  template class FactorNode<6, SE3PoseVel, SE3Pose>;     // SE3PoseVelPose
  template class FactorNode<3, SE3PoseVel, SE3Point>;    // SE3PoseVelPoint

  // =============================================================================
  // FactorGraph::addVariableNode Instantiations
  // =============================================================================
  template void FactorGraph::addVariableNode<SE3PoseVel>(const std::shared_ptr<SE3PoseVel>&);
  template void FactorGraph::addVariableNode<SE3Pose>(const std::shared_ptr<SE3Pose>&);
  template void FactorGraph::addVariableNode<SE3Point>(const std::shared_ptr<SE3Point>&);

  // =============================================================================
  // FactorGraph::addFactorNode Instantiations
  // =============================================================================
  template void FactorGraph::addFactorNode<SE3PoseVelPrior>(const std::shared_ptr<SE3PoseVelPrior>&, std::vector<std::string>);
  template void FactorGraph::addFactorNode<SE3PoseVelPoseVel>(const std::shared_ptr<SE3PoseVelPoseVel>&,
                                                              std::vector<std::string>);
  template void FactorGraph::addFactorNode<SE3PoseVelGP>(const std::shared_ptr<SE3PoseVelGP>&, std::vector<std::string>);
  template void FactorGraph::addFactorNode<SE3PoseVelPose>(const std::shared_ptr<SE3PoseVelPose>&, std::vector<std::string>);
  template void FactorGraph::addFactorNode<SE3PoseVelPoint>(const std::shared_ptr<SE3PoseVelPoint>&, std::vector<std::string>);

} // namespace gsolver
