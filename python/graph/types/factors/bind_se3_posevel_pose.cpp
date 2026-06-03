// =============================================================================
// gsolver Python bindings - SE3PoseVelPose factor (pose-to-landmark)
// =============================================================================

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <nanobind/eigen/dense.h>

#include <gsolver/graph/types/factors/se3_posevel_pose.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_se3_posevel_pose(nb::module_& m) {
      nb::class_<SE3PoseVelPose, FactorNodeBase>(m, "SE3PoseVelPose", "Factor between SE3PoseVel and SE3Pose (landmark)")
        .def(
          "__init__",
          [](SE3PoseVelPose* self,
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
            new (self) SE3PoseVelPose(id, iso, sigma_eigen);
          },
          nb::arg("id"),
          nb::arg("z"),
          nb::arg("sigma"),
          "Create pose-to-landmark factor\n\n"
          "Args:\n"
          "    id: Unique identifier\n"
          "    z: Relative transformation to landmark as 4x4 matrix\n"
          "    sigma: 6x6 measurement covariance")
        .def_prop_rw(
          "z",
          [](const SE3PoseVelPose& f) -> Eigen::Matrix4d { return f.z_.matrix(); },
          [](SE3PoseVelPose& f, const Eigen::Matrix4d& m) {
            Eigen::Isometry3d iso;
            iso.matrix() = m;
            f.z_         = iso;
          },
          "Measurement as 4x4 transformation matrix");
    }

  } // namespace python
} // namespace gsolver
