// =============================================================================
// gsolver Python bindings - SE3PoseVelGP factor (Gaussian Process motion prior)
// =============================================================================

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

#include <nanobind/eigen/dense.h>

#include <gsolver/graph/types/factors/se3_posevel_gp.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    void bind_se3_posevel_gp(nb::module_& m) {
      nb::class_<SE3PoseVelGP, FactorNodeBase>(m, "SE3PoseVelGP", "Gaussian Process motion prior factor")
        .def(
          "__init__",
          [](SE3PoseVelGP* self, const std::string& id, double dt, nb::ndarray<double, nb::shape<12, 12>> sigma) {
            Eigen::Matrix<double, 12, 12> sigma_eigen;
            for (int i = 0; i < 12; ++i) {
              for (int j = 0; j < 12; ++j) {
                sigma_eigen(i, j) = sigma(i, j);
              }
            }
            new (self) SE3PoseVelGP(id, dt, sigma_eigen);
          },
          nb::arg("id"),
          nb::arg("dt"),
          nb::arg("sigma"),
          "Create GP motion prior factor\n\n"
          "Args:\n"
          "    id: Unique identifier\n"
          "    dt: Time difference between poses\n"
          "    sigma: 12x12 process noise covariance")
        .def_rw("dt", &SE3PoseVelGP::dt_, "Time difference");
    }

  } // namespace python
} // namespace gsolver
