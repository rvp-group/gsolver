// =============================================================================
// gsolver Python bindings - SE3Pose variable
// =============================================================================

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <nanobind/eigen/dense.h>

#include <gsolver/graph/types/variables/se3_pose.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_se3_pose(nb::module_& m) {
      nb::class_<SE3Pose, VariableNodeBase>(m, "SE3Pose", "SE3 pose variable node")
        .def(
          "__init__",
          [](SE3Pose* self,
             const std::string& id,
             nb::ndarray<double, nb::shape<4, 4>> mu,
             nb::ndarray<double, nb::shape<6, 6>> sigma) {
            Eigen::Matrix4d mu_eigen;
            Eigen::Matrix<double, 6, 6> sigma_eigen;
            for (int i = 0; i < 4; ++i) {
              for (int j = 0; j < 4; ++j) {
                mu_eigen(i, j) = mu(i, j);
              }
            }
            for (int i = 0; i < 6; ++i) {
              for (int j = 0; j < 6; ++j) {
                sigma_eigen(i, j) = sigma(i, j);
              }
            }
            Eigen::Isometry3d iso;
            iso.matrix() = mu_eigen;
            new (self) SE3Pose(id, iso, sigma_eigen);
          },
          nb::arg("id"),
          nb::arg("mu"),
          nb::arg("sigma"),
          "Create SE3Pose variable\n\n"
          "Args:\n"
          "    id: Unique identifier\n"
          "    mu: Initial pose estimate as 4x4 transformation matrix\n"
          "    sigma: 6x6 covariance matrix")
        .def_ro("id", &SE3Pose::id_, "Variable node ID")
        .def_prop_rw(
          "mu",
          [](const SE3Pose& v) -> Eigen::Matrix4d { return v.mu_.matrix(); },
          [](SE3Pose& v, const Eigen::Matrix4d& m) {
            Eigen::Isometry3d iso;
            iso.matrix() = m;
            v.mu_        = iso;
          },
          "Current pose estimate as 4x4 transformation matrix")
        .def_rw("sigma", &SE3Pose::Sigma_, "Covariance matrix");
    }

  } // namespace python
} // namespace gsolver
