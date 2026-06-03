// =============================================================================
// gsolver Python bindings - FactorNodeBase
// =============================================================================
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <gsolver/graph/factor_node_base.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_factor_node_base(nb::module_& m) {
      nb::class_<FactorNodeBase>(m, "FactorNodeBase", "Base class for factor nodes")
        .def_ro("id", &FactorNodeBase::id_, "Factor node ID")
        .def(
          "set_edge_at",
          &FactorNodeBase::setEdgeAt,
          nb::arg("index"),
          nb::arg("edge"),
          "Associate an edge with the factor at the given slot")
        .def(
          "update_factor",
          &FactorNodeBase::updateFactor,
          "Recompute the factor's canonical parameters from the connected edges")
        .def(
          "send_message",
          &FactorNodeBase::sendMessage,
          nb::arg("variable_id"),
          "Send a message toward the specified variable node")
        .def(
          "send_all_messages",
          &FactorNodeBase::sendAllMessages,
          "Broadcast messages to every connected variable node");
    }

  } // namespace python
} // namespace gsolver
