// =============================================================================
// gsolver Python bindings - Main module entry point
// =============================================================================

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h> // Required for std::string conversion in all bindings

namespace nb = nanobind;

// Forward declarations of binding functions
namespace gsolver::python {
  void bind_gbp_solver(nb::module_& m);
  void bind_geometry3d(nb::module_& m);
  void bind_variable_node_base(nb::module_& m);
  void bind_factor_node_base(nb::module_& m);
  void bind_edge_base(nb::module_& m);
  void bind_edge(nb::module_& m);
  void bind_factor_graph(nb::module_& m);
  void bind_se3_point(nb::module_& m);
  void bind_se3_pose(nb::module_& m);
  void bind_se3_posevel(nb::module_& m);
  void bind_se3_posevel_prior(nb::module_& m);
  void bind_se3_posevel_posevel(nb::module_& m);
  void bind_se3_posevel_gp(nb::module_& m);
  void bind_se3_posevel_pose(nb::module_& m);
  void bind_se3_posevel_point(nb::module_& m);
} // namespace gsolver::python

NB_MODULE(_gsolver, m) {
  m.doc() = R"pbdoc(
        gsolver - Gaussian Belief Propagation SLAM Library
        ====================================================

        A C++ library for solving SLAM problems using Gaussian Belief Propagation.

        Submodules:
            core: Solver and optimization algorithms
            graph: Factor graph, variables, and factors
            math: Geometry utilities (SE3, SO3 operations)

        Example:
            >>> import gsolver
            >>> from gsolver.core import GbpSolver, SolverScheduleType
            >>> from gsolver.graph import FactorGraph, SE3PoseVel, SE3PoseVelPrior
            >>> 
            >>> # Create factor graph
            >>> fg = FactorGraph()
            >>> 
            >>> # Add variables and factors...
            >>> 
            >>> # Solve
            >>> solver = GbpSolver(SolverScheduleType.SYNCHRONOUS)
            >>> solver.solve(fg)
    )pbdoc";

  // Math module bindings
  gsolver::python::bind_geometry3d(m);

  // Graph module - base classes (must be bound before derived classes)
  gsolver::python::bind_variable_node_base(m);
  gsolver::python::bind_factor_node_base(m);
  gsolver::python::bind_edge_base(m);

  // Graph module - variable types
  gsolver::python::bind_se3_point(m);
  gsolver::python::bind_se3_pose(m);
  gsolver::python::bind_se3_posevel(m);

  // Graph module - typed edges (require variable types to be registered)
  gsolver::python::bind_edge(m);

  // Graph module - factor types
  gsolver::python::bind_se3_posevel_prior(m);
  gsolver::python::bind_se3_posevel_posevel(m);
  gsolver::python::bind_se3_posevel_gp(m);
  gsolver::python::bind_se3_posevel_pose(m);
  gsolver::python::bind_se3_posevel_point(m);

  // Graph module - factor graph (depends on variable and factor types)
  gsolver::python::bind_factor_graph(m);

  // Core module bindings
  gsolver::python::bind_gbp_solver(m);
}
