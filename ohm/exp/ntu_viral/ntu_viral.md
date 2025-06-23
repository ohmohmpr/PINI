# NTU_VIRAL

## TABLE

### Collected at the School of EEE central carpark

outdoor but smaller area.

| seq   | topic  | result  |         frame    | Loop corrected |     BUG/info         |
| ---   |  ----  | ------  | ---------------- | -------------- | -------------------- |
|eee_01 | topic 1| success | 4094/4094        |                |                      |
|eee_02 | topic 1| success | 3210/3210, 09:00 |                |                      |

### Collected inside the Nanyang Auditorium

[EASY] indoor, The building with a big stair.

| seq   | topic  | result  |         frame    | Loop corrected |     BUG/info         |
| ---   |  ----  | ---     | ---------------- | -------------- | -------------------- |
|nya_03 | topic 1| success | 4094/4094, 08:36 |                |                      |

### Collected at the Research Techno Plaza's carpark

| seq   | topic  | result  |       frame      |   Loop corrected   |        BUG/info      |
| ---   |  ----  | ---     | ---------------- | ------------------ | -------------------- |
|rtp_01 | topic 1| success | 4616/4616, 19:40 | Loop corrected:  6 |  BUG 2, sometimes?   |
|rtp_02 | topic 1| success | 4153/4153, 14:03 |                    |                      |
|rtp_03 | topic 1| success | 3557/3557, 11:24 | -                  |                      |

### Collected at the School of Bio. Science's front square

[EASY] outside but slowly rotate

| seq   | topic  | result  |         frame    | Loop corrected |     BUG/info         |
| ---   |  ----  | ---     | ---------------- | -------------- | -------------------- |
|sbs_01 | topic 1| success | 4573/4573, 14:40 |                | BUG whileloop of sensor |

### School of Physical and Mathematical Science's Facade

[HARD] Flying in front of buildings and trees.

| seq   | topic  | result  |       frame      |   Loop corrected   |        BUG/info      |
| ---   |  ----  | ---     | ---------------- | ------------------ | -------------------- |
|spms_01| topic 1| success | 4183/4183 20:00  |                    |                      |
|spms_02| topic 1| success | 3653/3653        |                    |                      |
|spms_03| topic 1| success | 3819/3819, 11:56 |                    |                      |

### Collected inside Research Techno Plaza

[EASY] UAV slowly rotate that's why this thing, cycle stair in the building.

| seq   | topic  | result  |         frame    | Loop corrected |    BUG/info   |
| ---   |  ----  | ---     | ---------------- | -------------- | ------------- |
|tnp_02 | topic 1| success | 4573/4573, 14:40 |                |               |
|tnp_03 | topic 1| success | 4078/4078, 12:35 |                |               |

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

## [BUG 2]

```bash
# Traceback (most recent call last):
#   File "pin_slam.py", line 673, in <module>
#     run_pin_slam()
#   File "pin_slam.py", line 355, in run_pin_slam
#     neural_points.reset_local_map(local_map_pose[:3,3], None, local_map_frame_id, False, config.loop_local_map_time_window)
#   File "/home/ohmpr/master_bonn/Modules/thesis/Pinocchio/model/neural_points.py", line 422, in reset_local_map
#     local_mask_idx = time_mask_idx[dist_mask] # True index
# IndexError: too many indices for tensor of dimension 0
```
