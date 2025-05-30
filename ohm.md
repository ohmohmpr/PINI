# LIO_EKF + PIN_SLAM

## How to install

```bash
## you must install conda with pin-enviroment first.
pip3 install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake jupyterlab
## for development, took it from kiss-icp
pip3 uninstall LIOEKF_pybind -y && CMAKE_POLICY_VERSION_MINIMUM=3.15 pip3 install --no-build-isolation -ve dependencies
```

## How run

### 1. M2DGR

```bash
python3 pin_slam.py ./config/lidar_slam/run_m2dgr.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/street_03/ -dv
```

### 2. NTU_VIRAL

```bash
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_01/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_02/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/nya_03/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node2/points -i ~/data/NTU_VIRAL/nya_03/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_01/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_02/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_03/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_01/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_02/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_03/ -d /
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_02/ -dv 
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_03/ -dv 
```

### 3. newer_college_dataset

```bash

# 64beams-> short
python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/01_short_experiment/rosbag/rooster_2020-03-10-11-36-51_0.bag -dv

# 64beams -> long
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/02_long_experiment/rosbag/rooster_2020-03-10-12-04-41_10.bag -dv

# 64beams -> 5_quad_dynamics
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/05_quad_with_dynamics/rosbag/rooster_2020-07-10-09-16-39_1-001.bag -dv

# 64beams -> 6_dynamic_spinning
python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/06_dynamic_spinning/rosbag/rooster_2020-07-10-09-23-18_0.bag -dv

# # 128beams collection 1
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-10-37-38-quad-easy-002.bag -dv
## too hard -- 700 lio-ekf, 1605 for pin-slam
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-11-31-35_0-quad-medium-004.bag -dv
## too DAMN hard -> delete
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-11-35-14_0-quad-hard-001.bag -dv
## too DAMN hard -> delete
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-10-40-50_0-stairs-005.bag -dv

# # 128beams collection 2
# good example for showing when turning, the lio-ekf failed
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-12-02-10-19-05_1-cloister.bag -dv
# good example for going to new space and fail, the lio-ekf failed
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-09-49_0-park.bag -dv
# repetition -> delete
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-26-36_5-park.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-30-06_6-park.bag  -dv 
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-33-19_7-park.bag -dv


# # collection 3
# the lio-ekf failed and pin-slam
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_3_maths_institute/2021-04-07-13-58-54_0-math-hard.bag -dv


# # collection 4
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_4_underground_mine/2021-04-12-11-06-47-easy.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_4_underground_mine/2021-04-12-11-24-30-hard.bag -dv
```

### 4. urbanNav

```bash
python3 pin_slam.py ./config/lidar_slam/run_urbanNav.yaml rosbag_ohm /velodyne_points -i ~/data/urbanNav/ -dv
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