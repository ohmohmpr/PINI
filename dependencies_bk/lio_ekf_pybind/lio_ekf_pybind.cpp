// #include "lio_ekf_core/lio_ekf.hpp"
#include "LIO_EKF/src/lio_ekf.hpp"
// #include "lio_ekf_core/lio_types.hpp"
#include "stl_vector_eigen.h"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

PYBIND11_MODULE(LIOEKF_pybind, m) {
  auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
      m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
      py::py_array_to_vectors_double<Eigen::Vector3d>);

  m.def("_imuCompensate", &lio_ekf::imuCompensate, "imu"_a, "ImuError"_a);

  py::class_<lio_ekf::IMU>(m, "_IMU").def(
      py::init<double, double, Eigen::Vector3d, Eigen::Vector3d>(),
      "timestamp"_a, "dt"_a, "angular_velocity"_a, "linear_acceleration"_a);

  py::class_<lio_ekf::ImuError>(m, "_ImuError")
      .def(py::init<>())
      .def_readwrite("gyrbias",
                     &lio_ekf::ImuError::gyrbias) // Eigen::Vector3d
      .def_readwrite("accbias",
                     &lio_ekf::ImuError::accbias); // Eigen::Vector3d

  py::class_<lio_ekf::ImuNoise>(m, "_ImuNoise")
      .def(py::init<>())
      .def_readwrite("angle_randomwalk",
                     &lio_ekf::ImuNoise::angle_randomwalk) // Eigen::Vector3d
      .def_readwrite("velocity_randomwalk",
                     &lio_ekf::ImuNoise::velocity_randomwalk) // Eigen::Vector3d
      .def_readwrite("gyrbias_std",
                     &lio_ekf::ImuNoise::gyrbias_std) // Eigen::Vector3d
      .def_readwrite("accbias_std",
                     &lio_ekf::ImuNoise::accbias_std) // Eigen::Vector3d
      .def_readwrite("correlation_time",
                     &lio_ekf::ImuNoise::correlation_time); // Eigen::Vector3d

  py::class_<lio_ekf::NavState>(m, "_NavState")
      .def(py::init<>())
      .def_readwrite("pos",
                     &lio_ekf::NavState::pos) // Eigen::Vector3d
      .def_readwrite("vel",
                     &lio_ekf::NavState::vel) // Eigen::Vector3d
      .def_readwrite("euler",
                     &lio_ekf::NavState::euler) // Eigen::Vector3d
      .def_readwrite("imuerror",
                     &lio_ekf::NavState::imuerror); // lio_ekf::ImuError

  py::class_<lio_ekf::LIOPara>(m, "_LIOPara", py::dynamic_attr())
      .def(py::init<>())
      .def_readwrite("deskew", &lio_ekf::LIOPara::deskew)         // bool
      .def_readwrite("preprocess", &lio_ekf::LIOPara::preprocess) // bool
      .def_readwrite("max_range", &lio_ekf::LIOPara::max_range)   // float
      .def_readwrite("min_range", &lio_ekf::LIOPara::min_range)   // float
      .def_readwrite("max_points_per_voxel",
                     &lio_ekf::LIOPara::max_points_per_voxel)           // int
      .def_readwrite("voxel_size", &lio_ekf::LIOPara::voxel_size)       // float
      .def_readwrite("max_iteration", &lio_ekf::LIOPara::max_iteration) // int

      .def_readwrite("initstate_std", &lio_ekf::LIOPara::initstate_std)
      .def_readwrite("imunoise", &lio_ekf::LIOPara::imunoise)

      // extrinsic parameters between lidar and imu Trans_lidar_imu,
      .def_readwrite(
          "Trans_lidar_imu",
          &lio_ekf::LIOPara::Trans_lidar_imu) // Eigen::Matrix4d
                                              // Eigen::Matrix4d::Identity()
      .def_readwrite(
          "imu_tran_R",
          &lio_ekf::LIOPara::imu_tran_R) // Eigen::Matrix4d
                                         // Eigen::Matrix4d::Identity()
      .def_readwrite("Trans_lidar_imu_origin",
                     &lio_ekf::LIOPara::
                         Trans_lidar_imu_origin); // Eigen::Matrix4d
                                                  // Eigen::Matrix4d::Identity()

  py::class_<lio_ekf::LIOEKF>(m, "_LIOEKF")
      .def(py::init<lio_ekf::LIOPara &>(), "LIOPara"_a)
      .def(py::init<>())
      //   .def("_getLIOPara", &lio_ekf::LIOEKF::getLIOPara)
      .def("_init", &lio_ekf::LIOEKF::init)
      .def("_addImuData", &lio_ekf::LIOEKF::addImuData)
      .def("_addLidarData", &lio_ekf::LIOEKF::addLidarData)
      .def("_getImutimestamp", &lio_ekf::LIOEKF::getImutimestamp)
      .def("_TransformPoints", &lio_ekf::LIOEKF::TransformPoints)
      .def("_newImuProcess", &lio_ekf::LIOEKF::newImuProcess)
      .def("_getNavState", &lio_ekf::LIOEKF::getNavState)
      // Map
      .def("_LocalMap", &lio_ekf::LIOEKF::LocalMap)
      .def("_getFrame_w", &lio_ekf::LIOEKF::getFrame_w)
      .def("_getKetPoints_w", &lio_ekf::LIOEKF::getKetPoints_w)
      .def("_getPoint_w", &lio_ekf::LIOEKF::getPoint_w)
      .def("_openResults", &lio_ekf::LIOEKF::openResults)
      .def("_writeResults", &lio_ekf::LIOEKF::writeResults)
      .def("_publishMsgs", &lio_ekf::LIOEKF::publishMsgs)
      .def("_getCovariance", &lio_ekf::LIOEKF::getCovariance)
      .def_readwrite("lidar_updated_",
                     &lio_ekf::LIOEKF::lidar_updated_) // Eigen::Matrix4d
      .def_readwrite("newpose",
                     &lio_ekf::LIOEKF::newpose); // Eigen::Matrix4d
}
