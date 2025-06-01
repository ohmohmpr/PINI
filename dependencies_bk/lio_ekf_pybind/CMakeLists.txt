
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
find_package(Eigen3 REQUIRED)


include(LIO_EKF/kiss-icp.cmake)
include(LIO_EKF/cmake/eigen.cmake)
include(LIO_EKF/cmake/sophus.cmake)

# add_subdirectory(std_msgs)

add_library(lio_ekf_core STATIC)
target_sources(lio_ekf_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/LIO_EKF/src/lio_ekf.cpp
                                    ${CMAKE_CURRENT_SOURCE_DIR}/LIO_EKF/src/imuPropagation.cpp)
target_include_directories(lio_ekf_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/LIO_EKF/src)

target_link_libraries(lio_ekf_core PUBLIC
                      kiss_icp::core 
                      Eigen3::Eigen)

pybind11_add_module(LIOEKF_pybind MODULE lio_ekf_pybind.cpp)
target_link_libraries(LIOEKF_pybind PRIVATE lio_ekf_core)
target_include_directories(lio_ekf_core PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/LIO_EKF/src)
install(TARGETS LIOEKF_pybind LIBRARY DESTINATION .)