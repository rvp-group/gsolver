#pragma once

#include <gsolver/graph/factor_graph.h>

namespace gsolver {

  enum class SolverScheduleType { SYNCHRONOUS, ASYNCHRONOUS };

  class GbpSolver {
  public:
    GbpSolver(SolverScheduleType schedule_type);

    SolverScheduleType schedule_type_;

    void performIteration(FactorGraph& factor_graph);
    void performAttentionIteration(FactorGraph& factor_graph,
                                   const std::vector<std::string>& attention_variables_ids,
                                   const std::vector<std::string>& attention_factors_ids);
    void solve(FactorGraph& factor_graph);
  };

} // namespace gsolver
