import numpy as np
from tqdm import tqdm

from utils.config import Config

class SensorFusionManager:
    """
    in case unsynchronize data sets.
    next of class generator can not go back, therefore load them independenly is a good idea.
    but come with a trade-off of function np.argmin.
    working like a queue that generate a stream of data.
    """
    def __init__(self, config: Config, loader):
        self.config = config
        self.loader = loader
        self.imus = loader.imus
        self.imu_ROSIMU_list = [self.imus[imu_topic_k] for _, imu_topic_k in enumerate(self.imus)]

        self.tqdm_bars = {}
        self.imu_manager_dict = {}
        # self.file = open("SensorFusionManager.txt", "w+")

        # self._start_tqdm()
        self._start_imu_manager_dict()

    def __len__(self):
        num = 0
        for _, sensor_type in enumerate(self.config.sensor_fusion["sensor_types"]):
            num = num + len(self.config.sensor_fusion["sensor_types"][sensor_type])

        return num

    def _get_len_sensor_types(self):
        return len(self.config.sensor_fusion["sensor_types"])
    
    def _start_tqdm(self):
        position = 1
        for i, imu_topic in enumerate(self.imus):
            self.tqdm_bars[imu_topic] = tqdm(total=len(self.imus[imu_topic]), position=position+i)

    def _start_imu_manager_dict(self):
        # config should transfer into object here
        for i, imu_topic in enumerate(self.imus):
            self.imu_manager_dict[imu_topic] = self.IMUManager(self.config,
                                                          self.loader,
                                                          self.imus[imu_topic])
    
    def get_min(self, timestamp_head_main_sensor):
        self.imus_list_timestamp_head = [self.imus[imu_topic].timestamp_head for _, imu_topic in enumerate(self.imus)]
        min_x = np.argmin([timestamp_head_main_sensor, *self.imus_list_timestamp_head])
        return min_x
    
    def _next(self, min_x):
        sensor_idx = min_x-1
        rosimu = self.imu_ROSIMU_list[sensor_idx]

        topic = rosimu.topic
        self.imu_manager_dict[topic].add(rosimu[rosimu.idx_ros_imu])
        # important
        # self.buffer.append(rosimu[rosimu.idx_ros_imu])

    def _write_ts(self, min_x):
        sensor_idx = min_x-1
        rosimu = self.imu_ROSIMU_list[sensor_idx]
        self.file.write(str(rosimu.idx_ros_imu) + f" {rosimu.topic} " 
                        + str(round(rosimu.timestamp_head, 10)))
        self.file.write("\n")

    def _update_bars(self, min_x):
        sensor_idx = min_x-1
        rosimu = self.imu_ROSIMU_list[sensor_idx]
        topic = rosimu.topic
        self.tqdm_bars[topic].update()
        self.tqdm_bars[topic].refresh()

    def _write_main_sensor(self, timestamp_head_main_sensor, frame_id):
        self.file.write(str(frame_id) + " lidar " + str(round(timestamp_head_main_sensor, 10)))
        self.file.write("\n")

    def get_latest_data(self, timestamp_head_main_sensor, frame_id):

        for sensor_idx in range(len(self.imu_ROSIMU_list)):
            rosimu = self.imu_ROSIMU_list[sensor_idx]
            topic = rosimu.topic
            self.imu_manager_dict[topic].buffer = []

        min_x = self.get_min(timestamp_head_main_sensor)
        while min_x != 0:
            min_x = self.get_min(timestamp_head_main_sensor)
            # self._write_ts(min_x)
            # data to the self
            self._next(min_x)
            # self._update_bars(min_x)
        # self._write_main_sensor(timestamp_head_main_sensor, frame_id)
        return None
    
    class IMUManager:
        def __init__(self, config: Config, loader, topic):
            self.config = config
            self.loader = loader
            self.topic = topic.topic

            self.start_ts = 0
            self.time_for_initStaticAlignment = 3 # sec
            self.is_initStaticAlignment = False # sec
            self.init_roll = 0 # rad
            self.init_pitch = 0 # rad
            self.init_gyro_mean = np.array([0, 0, 0], dtype='float64')
            self.init_acc_mean = np.array([0, 0, 0], dtype='float64')
            self.idx = 0
            for i in range(len(self.config.imu_topic)):
                if self.topic == self.config.imu_topic[i]["topic"]:
                    arr = np.array(self.config.imu_topic[i]["extrinsic_main_imu"])
                    if arr.size == 12:
                        extrinsic_main_self = arr.reshape(3, 4)
                        np.vstack((extrinsic_main_self, np.array([0, 0, 0, 1])))
                    elif arr.size == 16:
                        extrinsic_main_self = arr.reshape(4, 4)
                    else:
                        print("ERROR, calibration matrix is wrong.")
                        break
                    # print("sdfsad", extrinsic_main_self)
                    self.extrinsic_main_imu = extrinsic_main_self

            self.buffer = []

            # print("self.extrinsic_main_imu", self.extrinsic_main_imu)
            # print("self.loader", self.loader)
            # print("self.topic", self.topic)

            self.hz = 1/150
            self.prev_timestamp = 0
            self.curr_timestamp_head = 0

        def add(self, frame_data):

            self.curr_timestamp_head = frame_data["timestamp"]
            # print("==========topic==========", self.topic)
            # print("self.curr_timestamp_head", self.curr_timestamp_head)
            # print("self.prev_timestamp", self.prev_timestamp)
            dt = self.curr_timestamp_head - self.prev_timestamp
            # print("self.prev_timestamp < 1e-5", self.prev_timestamp < 1e-5)
            if dt < 1e-5 or self.prev_timestamp < 1e-5:
                # print("hz")
                # print("dt {0:.8f}".format(dt))
                # print("self.prev_timestamp", self.prev_timestamp)
                # print("self.curr_timestamp_head", self.curr_timestamp_head)
                dt = self.hz
            if dt > 1:
                print("error dt")
                return
            # print("dt", dt)
            self.prev_timestamp = self.curr_timestamp_head
            frame_data["dt"] = dt
            self.buffer.append(frame_data)
            # print(frame_data)
            if self.start_ts == 0:
                self.start_ts = frame_data["timestamp"]

            self.init_gyro_mean += frame_data["imu"][0]
            self.init_acc_mean += frame_data["imu"][1]
            self.idx = self.idx + 1

            if ((frame_data["timestamp"] - self.start_ts) > self.time_for_initStaticAlignment
                and self.is_initStaticAlignment == False):
                self.init_gyro_mean = self.init_gyro_mean / self.idx
                self.init_acc_mean = self.init_acc_mean / self.idx
                self.initStaticAlignment()
                print("self.topic", self.topic)
                print("self.idx", self.idx)
                print("self.init_roll_degree", self.init_roll_degree)
                print("self.init_pitch_degree", self.init_pitch_degree)
                self.is_initStaticAlignment = True

        def initStaticAlignment(self):

            init_acc_mean_homo = np.hstack((self.init_acc_mean, np.array([1])))
            init_acc_mean = self.extrinsic_main_imu @ init_acc_mean_homo

            self.init_roll = np.arctan2(-init_acc_mean[1], -init_acc_mean[2])
            self.init_pitch = np.arctan2(init_acc_mean[0], 
                                    np.sqrt(init_acc_mean[1] * init_acc_mean[1] +
                                            init_acc_mean[2] * init_acc_mean[2]))
            self.init_roll_degree = np.degrees(self.init_roll)
            self.init_pitch_degree = np.degrees(self.init_pitch)

