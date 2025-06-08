# NTU_VIRAL

## TABLE

### Collected inside Research Techno Plaza

[EASY] UAV slowly rotate that's why this thing, cycle stair in the building.

| seq   | topic  | result  |         frame    | Loop corrected |    BUG/info   |
| ---   |  ----  | ---     | ---------------- | -------------- | ------------- |
|tnp_02 | topic 1| success | 4573/4573, 14:40 |                | gross outlier |
|tnp_02 | topic 2| success | 4573/4573        |                |               |
|tnp_03 | topic 1| success | 4078/4078, 12:35 |                | gross outlier |
|tnp_03 | topic 2| Lose track for a long time, system failed | 1861/4078  | | |

### Collected inside the Nanyang Auditorium

[EASY] that building with stair

| seq   | topic  | result  | frame | Loop corrected | BUG/info |
| ---   |  ----  | ---     | ----- | ------------   | -------- |
|nya_03 | topic 1| success | 4094/4094, 12:50 |  | frame 2900, i think sometihng wrong in mtx extrinsics |
|nyu_03 | topic 2| success | 4095/4095 |  | |

### Collected at the School of Bio. Science's front square

[EASY] outside but slowly rotate

| seq   | topic  | result  |         frame    | Loop corrected |     BUG/info         |
| ---   |  ----  | ---     | ---------------- | -------------- | --------------------------- |
|sbs_01 | topic 1| success | 4573/4573, 14:40 |                | rotate at frame 3000, BUG 1 |
|sbs_01 | topic 2| success | 4573/4573        |                |                      |

### Collected at the School of EEE central carpark

[HARD]

| seq   | topic  |      result  | frame     | Loop corrected | BUG/info |
| ---   |  ----  | ------------ | -----     | ------------   | -------- |
|eee_01 | topic 1| on progress  | -/4094 |                | why ugly, point cloud doesn't align|
|eee_01 | topic 2| success      | 4095/4095 |                | |
|eee_02 | topic 1| on progress  | -/4094 |                | point cloud doesn't align at frame 2271 |
|eee_02 | topic 2| on progress  | 4095/4095 |                | |

### Collected at the Research Techno Plaza's carpark

| seq   | topic  | result  | frame | Loop corrected | BUG/info |
| ---   |  ----  | ---     | ----- | ------------   | -------- |
|rtp_01 | topic 1| success                                          | 4616/4616, 27:19 | Loop corrected:  20 |           |
|rtp_01 | topic 2| FAILED                                           |            /4615 |   | weird shape, shift too much |
|rtp_02 | topic 1| success                                          | 4153/4153, 14:03 |   | flying under the building.  |
|rtp_02 | topic 2| Lose track for a long time, system failed        |  898/4152        |   |      |
|rtp_03 | topic 1| success but last step failed                     | 3557/3557, 11:24 | - | BUG 1|
|rtp_03 | topic 2| success | 3556/3556 | Loop corrected:  13 |      |

### School of Physical and Mathematical Science's Facade

[HARD] Flying in front of buildings and trees.

| seq   | topic  | result  | frame | Loop corrected | BUG/info |
| ---   |  ----  | ---     | ----- | ------------   | -------- |
|spms_01| topic 1| Too large translation in one frame, system failed | 2058/4183 | | frame 2036, a bit shift of point cloud, UAV flying over the building and imu? |
|spms_01| topic 2| Lose track for a long time, system failed | 842/4183  | | |
|spms_02| topic 1| Too large translation in one frame, system failed| 606/3653 | | it supposed to go right, but pcl go left? trans_imu?|
|spms_02| topic 2| Lose track for a long time, system failed,| 85/3654   | | |
|spms_03| topic 1| on progress                                       | 3819/3819, 11:56 | | algorithm fine, but visualization failed, go left|
|spms_03| topic 2| Lose track for a long time, system failed |  429/3819 | | |

## [BUG 1]

```bash
# get_latest_data
### Traceback (most recent call last):
#   File "pin_slam.py", line 670, in <module>
#     run_pin_slam()
#   File "pin_slam.py", line 218, in run_pin_slam
#     if (dataset.sensor_fusion_manager.get_latest_data(dataset.loader.timestamp_head, frame_id) == None):
#   File "/home/ohmpr/master_bonn/Modules/thesis/Pinocchio/utils/sensor_fusion_manager.py", line 92, in get_latest_data
#     self._next(min_x)
#   File "/home/ohmpr/master_bonn/Modules/thesis/Pinocchio/utils/sensor_fusion_manager.py", line 58, in _next
#     self.imu_manager_dict[topic].add(rosimu[rosimu.idx_ros_imu])
#   File "/home/ohmpr/master_bonn/Modules/thesis/Pinocchio/dataset/dataloaders/rosbag_ohm.py", line 204, in __getitem__
#     connection, _, rawdata = next(self.msgs)
# StopIteration
```