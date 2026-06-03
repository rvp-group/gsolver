#pragma once

#include <gsolver/graph/edge_base.h>

class VariableNodeBase {
public:
  std::string id_; // Variable node ID

  virtual void sendMessage(size_t outedge_id) = 0;
  virtual void sendMessage(const std::string& factor_id) = 0;
  virtual void sendAllMessages()              = 0;
  virtual void updateMuAndSigma()             = 0;
};
