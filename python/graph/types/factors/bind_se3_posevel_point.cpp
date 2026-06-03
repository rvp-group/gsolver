// =============================================================================
// gsolver Python bindings - SE3PoseVelPoint factor (pose-to-point)
// =============================================================================

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <nanobind/eigen/dense.h>

#include <gsolver/graph/types/factors/se3_posevel_point.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_se3_posevel_point(nb::module_& m) {
      nb::class_<SE3PoseVelPoint, FactorNodeBase>(m, "SE3PoseVelPoint", "Factor between SE3PoseVel and SE3Point")
        .def(
          "__init__",
          [](SE3PoseVelPoint* self,
             const std::string& id,
             nb::ndarray<double, nb::shape<3>> z,
             nb::ndarray<double, nb::shape<3, 3>> sigma) {
            Eigen::Vector3d z_eigen;
            Eigen::Matrix3d sigma_eigen;
            for (int i = 0; i < 3; ++i) {
              z_eigen(i) = z(i);
              for (int j = 0; j < 3; ++j) {
                sigma_eigen(i, j) = sigma(i, j);
              }
            }
            new (self) SE3PoseVelPoint(id, z_eigen, sigma_eigen);
          },
          nb::arg("id"),
          nb::arg("z"),
          nb::arg("sigma"),
          "Create pose-to-point factor\n\n"
          "Args:\n"
          "    id: Unique identifier\n"
          "    z: Point observation in robot frame\n"
          "    sigma: 3x3 measurement covariance")
        .def_rw("z", &SE3PoseVelPoint::z_, "Point measurement");
    }

  } // namespace python
} // namespace gsolver
