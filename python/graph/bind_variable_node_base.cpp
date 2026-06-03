// =============================================================================
// gsolver Python bindings - VariableNodeBase
// =============================================================================
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <gsolver/graph/variable_node_base.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_variable_node_base(nb::module_& m) {
      nb::class_<VariableNodeBase>(m, "VariableNodeBase", "Base class for variable nodes")
        .def_ro("id", &VariableNodeBase::id_, "Variable node ID")
        .def(
          "send_message",
          static_cast<void (VariableNodeBase::*)(size_t)>(&VariableNodeBase::sendMessage),
          nb::arg("edge_index"),
          "Send a message along the specified outgoing edge (by index)")
        .def(
          "send_message_to_factor",
          static_cast<void (VariableNodeBase::*)(const std::string&)>(&VariableNodeBase::sendMessage),
          nb::arg("factor_id"),
          "Send a message toward the factor connected via the provided ID")
        .def(
          "send_all_messages",
          &VariableNodeBase::sendAllMessages,
          "Send messages along all connected edges")
        .def(
          "update_mu_sigma",
          &VariableNodeBase::updateMuAndSigma,
          "Recompute the variable belief by fusing incoming messages");
    }

  } // namespace python
} // namespace gsolver
