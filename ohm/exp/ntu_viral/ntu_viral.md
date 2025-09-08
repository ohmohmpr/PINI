# NTU_VIRAL

## TABLE

### Collected at the School of EEE central carpark

outdoor but smaller area.

| seq   | topic  | result  |         frame    | Loop corrected |     BUG/info         |
| ---   |  ----  | ------  | ---------------- | -------------- | -------------------- |
|eee_01 | topic 1| success | 3987/3987        |                | LIO-EKF(1299) |
|eee_02 | topic 1| success | 3210/3210, 09:00 |                | LIO-EKF, PINI |
|eee_03 | topic 1| success | 3210/3210, 09:00 |                | LIO-EKF, PINI|

### Collected inside the Nanyang Auditorium

indoor, The building with a big stair.

| seq   | topic  | result  |         frame    |  start frame   | Loop corrected |     BUG/info         |
| ---   |  ----  | ---     | ---------------- | -------------- | -------------- | -------------------- |
|nya_01 | topic 1| success | 3950/3950, 08:38 |                |                | [07.09] LIO-EKF, PINI|
|nya_02 | topic 1| success | 4286/4287, 09:57 |                |                | [07.09] LIO-EKF, PINI|
|nya_03 | topic 1| success | 4094/4094, 09:48 |        -1      |      3         | [07.09] LIO-EKF, PINI|

### Collected at the Research Techno Plaza's carpark

| seq   | topic  | result  |       frame      |  start frame   | Loop corrected |     BUG/info         |
| ---   |  ----  | ---     | ---------------- | -------------- | -------------- | -------------------- |
|rtp_01 | topic 1| success | 4433/4616, 19:40 |  0/set state   |              6 |[07.09]|
|rtp_02 | topic 1| success | 4153/4153, 14:03 |                |   8            |[07.09]               |
|rtp_03 | topic 1| success | 3557/3557, 11:24 |                |  11            |[07.09]|

### Collected at the School of Bio. Science's front square

outside but slowly rotate

| seq   | topic  | result  |         frame    | Loop corrected |     BUG/info         |
| ---   |  ----  | ---     | ---------------- | -------------- | -------------------- |
|sbs_01 | topic 1| success | 4573/4573, 14:40 |                | drift [07.09]|
|sbs_02 | topic 1| success | 3732/3732, 14:40 |                | drift [07.09]|
|sbs_01 | topic 1| success | 4573/4573, 14:40 |                | drift [07.09]|

### School of Physical and Mathematical Science's Facade

Flying in front of buildings and trees.

| seq   | topic  | result  |       frame      |   Loop corrected   |        BUG/info      |
| ---   |  ----  | ---     | ---------------- | ------------------ | -------------------- |
|spms_01| topic 1| success | 4183/4183 20:00  |         9          |[07.09] good before 1500(fly away), 2600 wrong|
|spms_02| topic 1| success | 3653/3653        |                    |[07.09] good before 1500(fly away), after1849 wrong, LIO_EKF failed|
|spms_03| topic 1| success | 3819/3819, 11:56 |                    |[07.09] good before 944, after|

### Collected inside Research Techno Plaza

UAV slowly rotate that's why this thing, cycle stair in the building.

| seq   | topic  | result  |         frame    |  start frame   | Loop corrected |     BUG/info         |
| ---   |  ----  | ---     | ---------------- | -------------- | -------------- | -------------------- |
|tnp_01 | topic 1| success | 5795/5795, 14:20 |                |                |[07.09] no out        |
|tnp_02 | topic 1| success | 4573/4573, 14:40 |        -1      |                |[07.09] outlier       |
|tnp_03 | topic 1| success | 4078/4078, 10:42 |                |                |[07.09] drift       |

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
