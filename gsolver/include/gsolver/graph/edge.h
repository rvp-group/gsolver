#pragma once

#include <gsolver/graph/edge_base.h>

namespace gsolver {

  template <typename VariableType_>
  struct Message {
    using VariableType = VariableType_;
    using EstimateType = typename VariableType::EstimateType;
    using MatrixLieDim = typename VariableType::MatrixLieDim;

    bool processed_ = false; // Flag indicating if the message has been processed
    MatrixLieDim Lambda_; // Lambda matrix (precision matrix of the factor)
    EstimateType mu_;     // Estimate of the variable
  };

  template <typename VariableType>
  class Edge : public EdgeBase {
  public:
    using VariableType_ = VariableType;
    using MessageType   = Message<VariableType>;

    MessageType variable_to_factor_message_;
    MessageType factor_to_variable_message_;
  };

} // namespace gsolver