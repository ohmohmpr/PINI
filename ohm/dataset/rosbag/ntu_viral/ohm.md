# Ohm run

python3 pin_slam.py ./config/lidar_slam/run_kitti.yaml kitti 22 -i /home/ohmpr/Downloads/dataset/ -vsmd

## TABLE

| dataset   | seq   | topic  | remark  | frame | Loop corrected | command |
| ------    | ---   |  ----  | ---     | ----- | ------------   | ------- |
| NTU_VIRAL |eee_01 | topic 1| success | 4094/4094 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/eee_01/ -vsmd |
| NTU_VIRAL |eee_01 | topic 2| success | 4095/4095 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/eee_01/ -vsmd |
| NTU_VIRAL |nya_03 | topic 1| success | 4094/4094 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/nya_03/ -vsmd |
| NTU_VIRAL |nyu_03 | topic 2| success | 4095/4095 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/nya_03/ -vsmd |
| NTU_VIRAL |rtp_01 | topic 1| success | 4616/4616 | Loop corrected:  20 | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_01/ -vsmd |
| NTU_VIRAL |rtp_01 | topic 2| Too large translation in one frame, system failed, failed | 344/4615 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/rtp_01/ -vsmd |
| NTU_VIRAL |rtp_02 | topic 1| success | 4153/4153 |  | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_02/ -vsmd |
| NTU_VIRAL |rtp_02 | topic 2| Lose track for a long time, system failed | 898/4152 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/rtp_02/ -vsmd |
| NTU_VIRAL |rtp_03 | topic 1| success | 3557/3557 | Loop corrected:  12 | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/rtp_03/ -vsmd |
| NTU_VIRAL |rtp_03 | topic 2| success | 3556/3556 | Loop corrected:  13 | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/rtp_03/ -vsmd |
| NTU_VIRAL |tnp_02 | topic 1| success | 4573/4573 |             | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_02/ -vsmd |
| NTU_VIRAL |tnp_02 | topic 2| success | 4573/4573 |             | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/tnp_02/ -vsmd |
| NTU_VIRAL |tnp_03 | topic 1| success | 4078/4078 |             | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/tnp_02/ -vsmd |
| NTU_VIRAL |tnp_03 | topic 2| Lose track for a long time, system failed | 1861/4078 |             | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/tnp_03/ -vsmd |
| NTU_VIRAL |spms_01 | topic 1| Lose track for a long time, system failed | 2221/4183 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_01/ -vsmd|
| NTU_VIRAL |spms_01 | topic 2| Lose track for a long time, system failed | 842/4183 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/spms_01/ -vsmd |
| NTU_VIRAL |spms_02 | topic 1| Lose track for a long time, system failed,| 1396/3653 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_02/ -vsmd|
| NTU_VIRAL |spms_02 | topic 2| Lose track for a long time, system failed,| 85/3654 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/spms_02/ -vsmd |
| NTU_VIRAL |spms_03 | topic 1| Lose track for a long time, system failed | 2154/3819 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node1/points -i ~/data/NTU_VIRAL/spms_03/ -vsmd|
| NTU_VIRAL |spms_03 | topic 2| Lose track for a long time, system failed |  429/3819 | | python3 pin_slam.py ./config/lidar_slam/run.yaml rosbag /os1_cloud_node2/points -i ~/data/NTU_VIRAL/spms_03/ -vsmd |


python3 pin_slam.py ./config/lidar_slam/run_kitti.yaml kitti 00 -i /home/ohmpr/data/KITTI -vsmd
