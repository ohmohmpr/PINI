# m2dgr

## TABLE

### Street_03

[EASY] garden.

|   seq    |     topic      | result  |         frame    | Loop corrected |    BUG/info   |
| -------  |  ------------  | ---     | ---------------- | -------------- | ------------- |
|street_03 | /handsfree/imu | success | 4573/4573, 14:40 |                | gross outlier |

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
