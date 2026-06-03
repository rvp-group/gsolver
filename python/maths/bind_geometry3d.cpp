// =============================================================================
#include <nanobind/nanobind.h>
// gsolver Python bindings - geometry3d
// =============================================================================

#include <nanobind/eigen/dense.h>

#include <gsolver/maths/geometry3d.h>

namespace nb = nanobind;

namespace gsolver {
  namespace python {

    // Wrapper functions to convert Isometry3d to/from 4x4 matrix for Python
    Eigen::Matrix4d exp_map_se3_matrix(const Eigen::Vector<double, 6>& x) {
      return expMapSE3(x).matrix();
    }

    Eigen::Vector<double, 6> log_map_se3_matrix(const Eigen::Matrix4d& T) {
      Eigen::Isometry3d iso;
      iso.matrix() = T;
      return logMapSE3(iso);
    }

    Eigen::Matrix<double, 6, 6> adjoint_se3_matrix(const Eigen::Matrix4d& T) {
      Eigen::Isometry3d iso;
      iso.matrix() = T;
      return adjoint_SE3(iso);
    }

    void bind_geometry3d(nb::module_& m) {
      // Rotation matrices
      m.def("rotation_x",
            &rotationX,
            nb::arg("angle"),
            "Create a rotation matrix around the X axis\n\n"
            "Args:\n"
            "    angle: Rotation angle in radians\n\n"
            "Returns:\n"
            "    3x3 rotation matrix");

      m.def("rotation_y",
            &rotationY,
            nb::arg("angle"),
            "Create a rotation matrix around the Y axis\n\n"
            "Args:\n"
            "    angle: Rotation angle in radians\n\n"
            "Returns:\n"
            "    3x3 rotation matrix");

      m.def("rotation_z",
            &rotationZ,
            nb::arg("angle"),
            "Create a rotation matrix around the Z axis\n\n"
            "Args:\n"
            "    angle: Rotation angle in radians\n\n"
            "Returns:\n"
            "    3x3 rotation matrix");

      // SO3 exponential and log maps
      m.def("exp_map_so3",
            &expMapSO3,
            nb::arg("omega"),
            "Exponential map from so(3) to SO(3)\n\n"
            "Args:\n"
            "    omega: 3D rotation vector (axis-angle representation)\n\n"
            "Returns:\n"
            "    3x3 rotation matrix");

      m.def("log_map_so3",
            &logMapSO3,
            nb::arg("R"),
            "Logarithmic map from SO(3) to so(3)\n\n"
            "Args:\n"
            "    R: 3x3 rotation matrix\n\n"
            "Returns:\n"
            "    3D rotation vector (axis-angle representation)");

      // SE3 exponential and log maps (using 4x4 matrix representation)
      m.def("exp_map_se3",
            &exp_map_se3_matrix,
            nb::arg("x"),
            "Exponential map from se(3) to SE(3)\n\n"
            "Args:\n"
            "    x: 6D vector [translation (3), rotation (3)]\n\n"
            "Returns:\n"
            "    4x4 transformation matrix");

      m.def("log_map_se3",
            &log_map_se3_matrix,
            nb::arg("T"),
            "Logarithmic map from SE(3) to se(3)\n\n"
            "Args:\n"
            "    T: 4x4 transformation matrix\n\n"
            "Returns:\n"
            "    6D vector [translation (3), rotation (3)]");

      // Utility functions
      m.def("skew",
            &skew,
            nb::arg("v"),
            "Create a skew-symmetric matrix from a 3D vector\n\n"
            "Args:\n"
            "    v: 3D vector\n\n"
            "Returns:\n"
            "    3x3 skew-symmetric matrix");

      m.def("hat",
            &hat,
            nb::arg("v"),
            "Hat operator (alias for skew)\n\n"
            "Args:\n"
            "    v: 3D vector\n\n"
            "Returns:\n"
            "    3x3 skew-symmetric matrix");

      m.def("vee",
            &vee,
            nb::arg("M"),
            "Vee operator - extract vector from skew-symmetric matrix\n\n"
            "Args:\n"
            "    M: 3x3 skew-symmetric matrix\n\n"
            "Returns:\n"
            "    3D vector");

      m.def("adjoint_se3",
            &adjoint_se3_matrix,
            nb::arg("T"),
            "Compute the adjoint representation of SE(3)\n\n"
            "Args:\n"
            "    T: 4x4 transformation matrix\n\n"
            "Returns:\n"
            "    6x6 adjoint matrix");
    }

  } // namespace python
} // namespace gsolver
