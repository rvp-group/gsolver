// =============================================================================
// gsolver Python bindings - SE3PoseVel variable
// =============================================================================

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <nanobind/eigen/dense.h>

#include <gsolver/graph/types/variables/se3_posevel.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_se3_posevel(nb::module_& m) {
      // SE3PoseVelValue - Value type for SE3PoseVel variable
      nb::class_<SE3PoseVelValue>(m, "SE3PoseVelValue", "Value type containing pose and velocity")
        .def(nb::init<>(), "Default constructor")
        .def_prop_rw(
          "pose",
          [](const SE3PoseVelValue& v) -> Eigen::Matrix4d { return v.pose.matrix(); },
          [](SE3PoseVelValue& v, const Eigen::Matrix4d& m) {
            Eigen::Isometry3d iso;
            iso.matrix() = m;
            v.pose       = iso;
          },
          "SE3 pose as 4x4 transformation matrix")
        .def_rw("vel", &SE3PoseVelValue::vel, "6D velocity vector [linear (3), angular (3)]")
        .def("__repr__", [](const SE3PoseVelValue& v) {
          std::ostringstream oss;
          oss << "SE3PoseVelValue(t=[" << v.pose.translation().transpose() << "], v=[" << v.vel.transpose() << "])";
          return oss.str();
        });

      // SE3PoseVel variable node
      nb::class_<SE3PoseVel, VariableNodeBase>(m, "SE3PoseVel", "SE3 pose with velocity variable node")
        .def(
          "__init__",
          [](SE3PoseVel* self,
             const std::string& id,
             const SE3PoseVelValue& mu,
             nb::ndarray<double, nb::shape<12, 12>> sigma,
             double timestamp) {
            Eigen::Matrix<double, 12, 12> sigma_eigen;
            for (int i = 0; i < 12; ++i) {
              for (int j = 0; j < 12; ++j) {
                sigma_eigen(i, j) = sigma(i, j);
              }
            }
            new (self) SE3PoseVel(id, mu, sigma_eigen, timestamp);
          },
          nb::arg("id"),
          nb::arg("mu"),
          nb::arg("sigma"),
          nb::arg("timestamp"),
          "Create SE3PoseVel variable\n\n"
          "Args:\n"
          "    id: Unique identifier\n"
          "    mu: Initial estimate (SE3PoseVelValue)\n"
          "    sigma: 12x12 covariance matrix\n"
          "    timestamp: Associated timestamp")
        .def_ro("id", &SE3PoseVel::id_, "Variable node ID")
        .def_rw("mu", &SE3PoseVel::mu_, "Current estimate")
        .def_rw("sigma", &SE3PoseVel::Sigma_, "Covariance matrix")
        .def_rw("timestamp", &SE3PoseVel::timestamp_, "Timestamp");
    }

  } // namespace python
} // namespace gsolver
