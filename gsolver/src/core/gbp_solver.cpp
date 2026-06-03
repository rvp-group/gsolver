#include <gsolver/core/gbp_solver.h>

namespace gsolver {

  GbpSolver::GbpSolver(SolverScheduleType schedule_type) : schedule_type_(schedule_type) {
  }

  void GbpSolver::performIteration(FactorGraph& factor_graph) {
    if (schedule_type_ == SolverScheduleType::SYNCHRONOUS) {
// Update factors
// std::cout << "Update Factors" << std::endl;
#pragma omp parallel for
      for (auto& factor_node : factor_graph.factor_nodes_) {
        factor_node->updateFactor();
      }
// Factor-to-variable messages
// std::cout << "Factor Messages" << std::endl;
#pragma omp parallel for
      for (auto& factor_node : factor_graph.factor_nodes_) {
        factor_node->sendAllMessages();
      }

// Update variables
// std::cout << "Update Variables" << std::endl;s
#pragma omp parallel for
      for (auto& variable_node : factor_graph.variable_nodes_) {
        variable_node->updateMuAndSigma();
      }

// Variable-to-factor messages
// std::cout << "Variable Messages" << std::endl;
#pragma omp parallel for
      for (auto& variable_node : factor_graph.variable_nodes_) {
        variable_node->sendAllMessages();
      }
    } else if (schedule_type_ == SolverScheduleType::ASYNCHRONOUS) {
      // Implement asynchronous schedule (e.g., random node selection)
      // ...
    }
  }

  void GbpSolver::performAttentionIteration(FactorGraph& factor_graph,
                                            const std::vector<std::string>& attention_variables_ids,
                                            const std::vector<std::string>& attention_factors_ids) {
    if (schedule_type_ == SolverScheduleType::SYNCHRONOUS) {
      // Update factors
      for (std::string factor_id : attention_factors_ids) {
        factor_graph.getFactorNode(factor_id)->updateFactor();
      }

      // Factor-to-variable messages
      for (std::string factor_id : attention_factors_ids) {
        factor_graph.getFactorNode(factor_id)->sendAllMessages();
      }

      // Update variables
      for (std::string variable_id : attention_variables_ids) {
        factor_graph.getVariableNode(variable_id)->updateMuAndSigma();
      }

      // Variable-to-factor messages
      for (std::string variable_id : attention_variables_ids) {
        factor_graph.getVariableNode(variable_id)->sendAllMessages();
      }
    } else if (schedule_type_ == SolverScheduleType::ASYNCHRONOUS) {
      // Implement asynchronous schedule with attention
      // ...
    }
  }

  void GbpSolver::solve(FactorGraph& factor_graph) {
    // TODO: Implement stopping criteria (e.g., convergence threshold, maximum iterations)
    while (true) { // FIXME: This is a bug, and should be replaced with a proper stopping criterion
      performIteration(factor_graph);
    }
  }

} // namespace gsolver