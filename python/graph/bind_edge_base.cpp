// =============================================================================
// gsolver Python bindings - EdgeBase
// =============================================================================
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <sstream>

#include <gsolver/graph/edge_base.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_edge_base(nb::module_& m) {
      nb::class_<EdgeBase>(m, "EdgeBase", "Base class for graph edges")
        .def(nb::init<>(), "Create an empty edge base")

        // Properties
        .def_prop_ro(
          "variable_id",
          [](const EdgeBase& edge) { return edge.variable_id_; },
          "Variable ID associated with the edge")
        .def_prop_ro(
          "factor_id",
          [](const EdgeBase& edge) { return edge.factor_id_; },
          "Factor ID associated with the edge")
        .def("__repr__", [](const EdgeBase& edge) {
          std::ostringstream oss;
          oss << "EdgeBase(variable_id='" << edge.variable_id_ << "', factor_id='" << edge.factor_id_ << "')";
          return oss.str();
        });
    }

  } // namespace python
} // namespace gsolver
