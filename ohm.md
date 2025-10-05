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
# python3 pin_slam.py ./config/lidar_slam/run_m2dgr.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/rotation_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_m2dgr.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/street_03/ -dv
python3 pin_slam.py ./config/lidar_slam/run_m2dgr.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/gate_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_m2dgr.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/hall_02/ -dv
```

### 2. NTU_VIRAL

```bash
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_02/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_03/ -dv

python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_02/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_03/ -dv

python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/nya_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/nya_02/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/nya_03/ -dv
# Now test on this
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_02/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_03/ -dv

# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_01/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_02/ -dv
# python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_03/ -dv

python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/sbs_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/sbs_02/ -dv
python3 pin_slam.py ./config/lidar_slam/run_ntu_viral.yaml rosbag_ohm /os1_cloud_node1/points -i ~/data/NTU_VIRAL/sbs_03/ -dv
```

### 3. newer_college_dataset

```bash

# 64beams-> short
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/01_short_experiment/rosbag/rooster_2020-03-10-11-36-51_0.bag -dv
python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/01/bin/0.bag -dv

# 64beams -> long
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/02_long_experiment/rosbag/0.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/02_long_experiment/rosbag/10.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/02_long_experiment/rosbag/10.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/02_long_experiment/rosbag/10.bag -dv

# 64beams -> 5_quad_dynamics
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/05_quad_with_dynamics/rosbag/rooster_2020-07-10-09-16-39_1-001.bag -dv

# 64beams -> 6_dynamic_spinning
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/06_dynamic_spinning/rosbag/rooster_2020-07-10-09-23-18_0.bag -dv


# 64beams -> 7
# python3 pin_slam.py ./config/lidar_slam/run_newer_college64.yaml rosbag_ohm /os1_cloud_node/points -i ~/data/newer_college_dataset/2020-ouster-os1-64-realsense/07/2.bag -dv

# # 128beams collection 1
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-10-37-38-quad-easy-002.bag -dv
#### 1605 for pin-slam
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-11-31-35_0-quad-medium-004.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-11-35-14_0-quad-hard.bag -dv
## failed
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_1_newer_college/2021-07-01-10-40-50_0-stairs.bag -dv

## 128beams collection 2
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-12-02-10-15-59_0-cloister.bag -dv
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-12-02-10-19-05_1-cloister.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-09-49_0-park.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-23-25_4-park.bag -dv
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_2_newer_college/2021-11-30-17-33-19_7-park.bag -dv

# # collection 3
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_3_maths_institute/2021-04-07-13-49-03_0-math-easy.bag -dv
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_3_maths_institute/2021-04-07-13-52-31_1-math-easy.bag -dv
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_3_maths_institute/2021-04-07-13-55-18-math-medium.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_3_maths_institute/2021-04-07-13-58-54_0-math-hard.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_3_maths_institute/2021-04-07-14-02-18_1-math-hard.bag -dv


### collection 4
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_4_underground_mine/2021-04-12-11-06-47-easy.bag -dv
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_4_underground_mine/2021-04-12-11-11-33-medium.bag -dv
python3 pin_slam.py ./config/lidar_slam/run_newer_college.yaml rosbag_ohm /os_cloud_node/points -i ~/data/newer_college_dataset/2021-ouster-os0-128-alphasense/collection_4_underground_mine/2021-04-12-11-24-30-hard.bag -dv
```

### 4. urbanNav

```bash
python3 pin_slam.py ./config/lidar_slam/run_urbanNav.yaml rosbag_ohm /velodyne_points -i ~/data/urbanNav/ -dv
```

### 5. Oxford spires

```bash
### belmheim_01
python3 pin_slam.py ./config/lidar_slam/run_oxford_spires.yaml rosbag_ohm /hesai/pandar -i ~/data/oxford_spires/blenheim_01/1710406700_2024-03-14-08-58-21_0.bag -dv
```

### 6. hilti 2023

```bash
# python3 pin_slam.py ./config/lidar_slam/run_hilti_2023.yaml rosbag_ohm /hesai/pandar -i ~/data/hitti2023/site1_handheld_1.bag -dv
# python3 pin_slam.py ./config/lidar_slam/run_hilti_2023.yaml rosbag_ohm /hesai/pandar -i ~/data/hitti2023/site3_handheld_1.bag -dv
```

### 7. KITTI

```bash
python3 pin_slam.py ./config/lidar_slam/run_kitti.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/gate_01/ -dv
python3 pin_slam.py ./config/lidar_slam/run_kitti.yaml kitti 00 -vsm
python3 pin_slam.py ./config/lidar_slam/run_m2dgr.yaml rosbag_ohm /velodyne_points -i ~/data/m2dgr/hall_02/ -dv
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

## BUG

if you have one imu, ts will be different.
