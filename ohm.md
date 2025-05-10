# LIO_EKF + PIN_SLAM

## How to install

```bash
## you must install conda with pin-enviroment first.
pip3 install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake jupyterlab
## for development, took it from kiss-icp
pip3 uninstall LIOEKF_pybind -y && CMAKE_POLICY_VERSION_MINIMUM=3.15 pip3 install --no-build-isolation -ve dependencies
```

## How run

```bash
python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/street_03/ -d
# python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/street_03/ -vsmd
# python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_01/ -vsmd
# python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /velodyne_points -i ~/data/m2dgr/street_03/ -vsmd
```

## Cloning LIO_EKF, HOW TO DO

```bash
mkdir dependencies/lio_ekf_pybind/
git clone git@github.com:YibinWu/LIO-EKF.git dependencies/lio_ekf_pybind/LIO_EKF
```

```bash
## copy 2 files
cp CMakeLists.txt pyproject.md dependencies/lio_ekf_pybind/
## copy 3 files
cp CMakeLists.txt lio_ekf_pybind.cpp stl_vector_eigen.h dependencies/lio_ekf_pybind/LIO_EKF
## copy cmake folder
cp cmake dependencies/lio_ekf_pybind/LIO_EKF
## commend out something related to ros packages in *.hpp *.cpp
```