// =============================================================================
// gsolver Python bindings - Edge
// =============================================================================
#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <gsolver/graph/edge.h>
#include <gsolver/graph/types/variables/se3_point.h>
#include <gsolver/graph/types/variables/se3_pose.h>
#include <gsolver/graph/types/variables/se3_posevel.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    namespace detail {

      // Convert estimates for message bindings. Default just forwards the value.
      template <typename EstimateType, typename Enable = void>
      struct MessageEstimateAdapter {
        using PyType = EstimateType;

        static PyType to_python(const EstimateType& value) {
          return value;
        }

        static EstimateType from_python(const PyType& value) {
          return value;
        }
      };

      // Eigen::Isometry3d needs conversion to Matrix4d for Python.
      template <>
      struct MessageEstimateAdapter<Eigen::Isometry3d> {
        using PyType = Eigen::Matrix4d;

        static PyType to_python(const Eigen::Isometry3d& value) {
          return value.matrix();
        }

        static Eigen::Isometry3d from_python(const PyType& matrix) {
          Eigen::Isometry3d iso;
          iso.matrix() = matrix;
          return iso;
        }
      };

    } // namespace detail

    template <typename VariableType>
    void bind_edge_type(nb::module_& m, const char* edge_name, const char* message_name) {
      using EdgeType    = Edge<VariableType>;
      using MessageType = typename EdgeType::MessageType;
      using MatrixType  = typename MessageType::MatrixLieDim;
      using MuAdapter   = detail::MessageEstimateAdapter<typename MessageType::EstimateType>;
      using PyMuType    = typename MuAdapter::PyType;

      nb::class_<MessageType>(m, message_name, "Gaussian message exchanged by an edge")
        .def(nb::init<>(), "Create an empty message")
        .def_prop_rw(
          "processed",
          [](const MessageType& msg) { return msg.processed_; },
          [](MessageType& msg, bool value) { msg.processed_ = value; },
          "Flag indicating if the message has been processed")
        .def_prop_rw(
          "Lambda",
          [](const MessageType& msg) { return msg.Lambda_; },
          [](MessageType& msg, const MatrixType& lambda) { msg.Lambda_ = lambda; },
          "Precision matrix of the message")
        .def_prop_rw(
          "mu",
          [](const MessageType& msg) { return MuAdapter::to_python(msg.mu_); },
          [](MessageType& msg, const PyMuType& value) { msg.mu_ = MuAdapter::from_python(value); },
          "Mean state carried by the message");

      nb::class_<EdgeType, EdgeBase>(m, edge_name, "Typed edge connecting a variable node and a factor node")
        .def(nb::init<>(), "Create an empty edge")
        .def_prop_rw(
          "variable_to_factor_message",
          [](EdgeType& edge) -> MessageType& { return edge.variable_to_factor_message_; },
          [](EdgeType& edge, const MessageType& message) { edge.variable_to_factor_message_ = message; },
          "Message maintained by the variable node",
          nb::rv_policy::reference_internal)
        .def_prop_rw(
          "factor_to_variable_message",
          [](EdgeType& edge) -> MessageType& { return edge.factor_to_variable_message_; },
          [](EdgeType& edge, const MessageType& message) { edge.factor_to_variable_message_ = message; },
          "Message maintained by the factor node",
          nb::rv_policy::reference_internal);
    }

    void bind_edge(nb::module_& m) {
      bind_edge_type<SE3Point>(m, "EdgeSE3Point", "SE3PointMessage");
      bind_edge_type<SE3Pose>(m, "EdgeSE3Pose", "SE3PoseMessage");
      bind_edge_type<SE3PoseVel>(m, "EdgeSE3PoseVel", "SE3PoseVelMessage");
    }

  } // namespace python
} // namespace gsolver
