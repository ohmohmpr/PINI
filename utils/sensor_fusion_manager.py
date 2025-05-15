import numpy as np
from tqdm import tqdm

from utils.config import Config

class SensorFusionManager:
    """
    in case unsynchronize data sets.
    next of class generator can not go back, therefore load them independenly is a good idea.
    but come with a trade-off of function np.argmin.
    """
    def __init__(self, config: Config, imus: dict):
        self.config = config
        self.imus = imus
        self.imu_ROSIMU_list = [self.imus[imu_topic_k] for _, imu_topic_k in enumerate(self.imus)]

        self.tqdm_bars = {}
        self.file = open("test.txt", "w+")

        self._start_tqdm()
        self.buffer = []

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

    
    def get_min(self, timestamp_head_main_sensor):
        self.imus_list_timestamp_head = [self.imus[imu_topic].timestamp_head for _, imu_topic in enumerate(self.imus)]
        min_x = np.argmin([timestamp_head_main_sensor, *self.imus_list_timestamp_head])
        return min_x
    
    def _next(self, min_x):
        sensor_idx = min_x-1
        rosimu = self.imu_ROSIMU_list[sensor_idx]
        # important
        self.buffer.append(rosimu[rosimu.idx_ros_imu])

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
        self.buffer = []
        min_x = self.get_min(timestamp_head_main_sensor)
        while min_x != 0:
            min_x = self.get_min(timestamp_head_main_sensor)
            self._write_ts(min_x)
            # data to the self
            self._next(min_x)
            self._update_bars(min_x)
        self._write_main_sensor(timestamp_head_main_sensor, frame_id)
        return None
    
    def init_imu_manager():
        return 
    