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
# python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/street_03/ -d
# python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/street_03/ -vsmd
# python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_01/ -vsmd
# python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /velodyne_points -i ~/data/m2dgr/street_03/ -vsmd
python3 pin_slam.py ./config/lidar_slam/run_m2dgr.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/street_03/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_01/ -d 
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_02/ -d

# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_01/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_02/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/nya_03/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_01/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_02/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_03/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_01/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_02/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_03/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_03/ -d 
```

### newer_college_dataset

```bash

# # collection 1
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-10-37-38-quad-easy-002.bag -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-10-40-50_0-stairs-005.bag -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-11-31-35_0-quad-medium-004.bag -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-11-35-14_0-quad-hard-001.bag -dv

# # collection 2
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-09-49_0-park.bag -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-26-36_5-park.bag -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-30-06_6-park.bag  -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-33-19_7-park.bag -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-12-02-10-19-05_1-cloister.bag -dv


# # collection 3
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_3_maths_institute/2021-04-07-13-58-54_0-math-hard.bag -dv \


# # collection 4
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_4_underground_mine/2021-04-12-11-06-47-easy.bag -dv \
# python3 pin_slam.py ./config/lidar_slam/run_ntu_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_4_underground_mine/2021-04-12-11-24-30-hard.bag -dv
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