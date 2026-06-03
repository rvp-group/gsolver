// =============================================================================
// gsolver Python bindings - SE3PoseVelPoseVel factor (odometry/relative pose)
// =============================================================================

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <nanobind/eigen/dense.h>

#include <gsolver/graph/types/factors/se3_posevel_posevel.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_se3_posevel_posevel(nb::module_& m) {
      nb::class_<SE3PoseVelPoseVel, FactorNodeBase>(m, "SE3PoseVelPoseVel", "Relative pose factor between two SE3PoseVel nodes")
        .def(
          "__init__",
          [](SE3PoseVelPoseVel* self,
             const std::string& id,
             nb::ndarray<double, nb::shape<4, 4>> z,
             nb::ndarray<double, nb::shape<6, 6>> sigma) {
            Eigen::Matrix4d z_eigen;
            Eigen::Matrix<double, 6, 6> sigma_eigen;
            for (int i = 0; i < 4; ++i) {
              for (int j = 0; j < 4; ++j) {
                z_eigen(i, j) = z(i, j);
              }
            }
            for (int i = 0; i < 6; ++i) {
              for (int j = 0; j < 6; ++j) {
                sigma_eigen(i, j) = sigma(i, j);
              }
            }
            Eigen::Isometry3d iso;
            iso.matrix() = z_eigen;
            new (self) SE3PoseVelPoseVel(id, iso, sigma_eigen);
          },
          nb::arg("id"),
          nb::arg("z"),
          nb::arg("sigma"),
          "Create relative pose factor\n\n"
          "Args:\n"
          "    id: Unique identifier\n"
          "    z: Relative transformation measurement as 4x4 matrix\n"
          "    sigma: 6x6 measurement covariance")
        .def_prop_rw(
          "z",
          [](const SE3PoseVelPoseVel& f) -> Eigen::Matrix4d { return f.z_.matrix(); },
          [](SE3PoseVelPoseVel& f, const Eigen::Matrix4d& m) {
            Eigen::Isometry3d iso;
            iso.matrix() = m;
            f.z_         = iso;
          },
          "Relative pose measurement as 4x4 transformation matrix");
    }

  } // namespace python
} // namespace gsolver
