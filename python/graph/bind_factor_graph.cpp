// =============================================================================
// gsolver Python bindings - FactorGraph
// =============================================================================
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/types/factors/se3_posevel_gp.h>
#include <gsolver/graph/types/factors/se3_posevel_point.h>
#include <gsolver/graph/types/factors/se3_posevel_pose.h>
#include <gsolver/graph/types/factors/se3_posevel_posevel.h>
#include <gsolver/graph/types/factors/se3_posevel_prior.h>
#include <gsolver/graph/types/variables/se3_point.h>
#include <gsolver/graph/types/variables/se3_pose.h>
#include <gsolver/graph/types/variables/se3_posevel.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_factor_graph(nb::module_& m) {
      nb::class_<FactorGraph>(m, "FactorGraph", "Factor graph for Gaussian Belief Propagation")
        .def(nb::init<>(), "Create an empty factor graph")

        // Add variable nodes
        .def(
          "add_variable",
          [](FactorGraph& fg, std::shared_ptr<SE3PoseVel> var) { fg.addVariableNode(var); },
          nb::arg("variable"),
          "Add an SE3PoseVel variable to the graph")
        .def(
          "add_variable",
          [](FactorGraph& fg, std::shared_ptr<SE3Pose> var) { fg.addVariableNode(var); },
          nb::arg("variable"),
          "Add an SE3Pose variable to the graph")
        .def(
          "add_variable",
          [](FactorGraph& fg, std::shared_ptr<SE3Point> var) { fg.addVariableNode(var); },
          nb::arg("variable"),
          "Add an SE3Point variable to the graph")

        // Add factor nodes
        .def(
          "add_factor",
          [](FactorGraph& fg, std::shared_ptr<SE3PoseVelPrior> factor, std::vector<std::string> var_ids) {
            fg.addFactorNode(factor, var_ids);
          },
          nb::arg("factor"),
          nb::arg("variable_ids"),
          "Add a prior factor to the graph")
        .def(
          "add_factor",
          [](FactorGraph& fg, std::shared_ptr<SE3PoseVelPoseVel> factor, std::vector<std::string> var_ids) {
            fg.addFactorNode(factor, var_ids);
          },
          nb::arg("factor"),
          nb::arg("variable_ids"),
          "Add a relative pose factor to the graph")
        .def(
          "add_factor",
          [](FactorGraph& fg, std::shared_ptr<SE3PoseVelGP> factor, std::vector<std::string> var_ids) {
            fg.addFactorNode(factor, var_ids);
          },
          nb::arg("factor"),
          nb::arg("variable_ids"),
          "Add a GP motion prior factor to the graph")
        .def(
          "add_factor",
          [](FactorGraph& fg, std::shared_ptr<SE3PoseVelPose> factor, std::vector<std::string> var_ids) {
            fg.addFactorNode(factor, var_ids);
          },
          nb::arg("factor"),
          nb::arg("variable_ids"),
          "Add a pose-to-landmark factor to the graph")
        .def(
          "add_factor",
          [](FactorGraph& fg, std::shared_ptr<SE3PoseVelPoint> factor, std::vector<std::string> var_ids) {
            fg.addFactorNode(factor, var_ids);
          },
          nb::arg("factor"),
          nb::arg("variable_ids"),
          "Add a pose-to-point factor to the graph")

        // Getters
        .def("get_variable", &FactorGraph::getVariableNode, nb::arg("id"), nb::rv_policy::reference, "Get a variable node by ID")
        .def("get_factor", &FactorGraph::getFactorNode, nb::arg("id"), nb::rv_policy::reference, "Get a factor node by ID")
        .def("get_pose_variables", &FactorGraph::getPoseVariableNodes, "Get all pose variable nodes sorted by timestamp")

        // Properties
        .def_prop_ro(
          "num_variables", [](const FactorGraph& fg) { return fg.variable_nodes_.size(); }, "Number of variable nodes")
        .def_prop_ro(
          "num_factors", [](const FactorGraph& fg) { return fg.factor_nodes_.size(); }, "Number of factor nodes")
        .def_ro("variable_nodes", &FactorGraph::variable_nodes_, "List of variable nodes")
        .def_ro("factor_nodes", &FactorGraph::factor_nodes_, "List of factor nodes");
    }

  } // namespace python
} // namespace gsolver
