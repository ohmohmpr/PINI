cmake_minimum_required(VERSION 3.15...3.26)
project(LIOEKF_pybind LANGUAGES CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_subdirectory(lio_ekf_pybind)
