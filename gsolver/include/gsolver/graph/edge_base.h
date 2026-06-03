#pragma once

#include <string>

namespace gsolver {

  class EdgeBase {
  public:
    virtual ~EdgeBase() = default;

    std::string variable_id_;
    std::string factor_id_;
  };

} // namespace gsolver
