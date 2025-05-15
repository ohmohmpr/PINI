# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
# 2024 Yue Pan
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import glob
import os
import sys
from pathlib import Path
from typing import Sequence

from tqdm import tqdm

import natsort


class RosbagDataset:
    def __init__(self, data_dir: Path, topic: str, imu_topic: list, *_, **__):
        """ROS1 / ROS2 bagfile dataloader.

        It can take either one ROS2 bag file or one or more ROS1 bag files belonging to a split bag.
        The reader will replay ROS1 split bags in correct timestamp order.

        TODO: Merge mcap and rosbag dataloaders into 1
        """
        try:
            from rosbags.highlevel import AnyReader
        except ModuleNotFoundError:
            print('rosbags library not installed, run "pip install -U rosbags"')
            sys.exit(1)

        from utils.point_cloud2 import read_point_cloud

        self.read_point_cloud = read_point_cloud
        if data_dir.is_file():
            self.sequence_id = os.path.basename(data_dir).split(".")[0]
            self.bag = AnyReader([data_dir])
        else:
            bagfiles = [Path(path) for path in glob.glob(os.path.join(data_dir, "*.bag"))]
            if len(bagfiles) > 0:
                self.sequence_id = os.path.basename(bagfiles[0]).split(".")[0]
                self.bag = AnyReader(bagfiles)
            else:
                self.sequence_id = os.path.basename(data_dir).split(".")[0]
                self.bag = AnyReader([data_dir])

        if len(self.bag.paths) > 1:
            print("Reading multiple .bag files in directory:")
            print("\n".join(natsort.natsorted([path.name for path in self.bag.paths])))

        self.bag.open()
        self.topic = self.check_topic(topic)
        self.n_scans = self.bag.topics[self.topic].msgcount

        # 
        connections = [x for x in self.bag.connections if x.topic == self.topic]
        self.msgs = self.bag.messages(connections=connections)
        connection, _, rawdata = next(self.msgs)
        msg = self.bag.deserialize(rawdata, connection.msgtype)
        _, _, point_cloud_timestamp = self.read_point_cloud(msg)
        self.timestamp_head = point_cloud_timestamp

        # limit connections to selected topic
        connections = [x for x in self.bag.connections if x.topic == self.topic]
        self.msgs = self.bag.messages(connections=connections)
        self.timestamps = []

        # create imus
        self.imus = {}
        for _, topic in enumerate(imu_topic):
            self.imus[topic] = self.ROSIMU(self.bag, topic)


    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        try:
            connection, timestamp, rawdata = next(self.msgs)
            # print("timestamp lidar", timestamp, idx)
            self.timestamps.append(self.to_sec(timestamp))
            msg = self.bag.deserialize(rawdata, connection.msgtype)

            points, point_ts, point_cloud_timestamp = self.read_point_cloud(msg)
            self.timestamp_head = point_cloud_timestamp
            # imus = self.hands_free_imu.get_imus_before(point_cloud_timestamp)
            frame_data = {"points": points, "point_ts": point_ts, "timestamp": point_cloud_timestamp}
        except StopIteration:
            return {"points": None, "point_ts": None, "timestamp": None}


        return frame_data

    def load_data_to_txt(self, txtfile):
        file = open(txtfile, "w+")
        for frame_id in tqdm(range(self.n_scans)):
            file.write(str(round(self[frame_id]["timestamp"], 10)))
            file.write("\n")

        file.close()

    @staticmethod
    def to_sec(nsec: int):
        return float(nsec) / 1e9

    def get_frames_timestamps(self) -> list:
        return self.timestamps

    def check_topic(self, topic: str) -> str:
        # Extract all PointCloud2 msg topics from the bagfile
        point_cloud_topics = [
            topic[0]
            for topic in self.bag.topics.items()
            if topic[1].msgtype == "sensor_msgs/msg/PointCloud2"
        ]

        def print_available_topics_and_exit():
            print("Select from the following topics:")
            print(50 * "-")
            for t in point_cloud_topics:
                print(f"{t}")
            print(50 * "-")
            sys.exit(1)

        if topic and topic in point_cloud_topics:
            return topic
        # when user specified the topic check that exists
        if topic and topic not in point_cloud_topics:
            print(
                f'[ERROR] Dataset does not containg any msg with the topic name "{topic}". '
                "Specify the correct topic name by python pin_slam.py path/to/config/file.yaml rosbag your/topic ... ..."
            )
            print_available_topics_and_exit()
        if len(point_cloud_topics) > 1:
            print(
                "Multiple sensor_msgs/msg/PointCloud2 topics available."
                "Specify the correct topic name by python pin_slam.py path/to/config/file.yaml rosbag your/topic ... ..."
            )
            print_available_topics_and_exit()

        if len(point_cloud_topics) == 0:
            print("[ERROR] Your dataset does not contain any sensor_msgs/msg/PointCloud2 topic")
        if len(point_cloud_topics) == 1:
            return point_cloud_topics[0]

    class ROSIMU:
        idx_ros_imu = 0
        def __init__(self, bag, topic):

            self.bag = bag
            self.topic = topic
            self.n_scans = self.bag.topics[topic].msgcount

            from utils.point_cloud2 import read_imu
            self.read_imu = read_imu

            self.timestamp_head = self._get_first_timestamp_head()

            # limit connections to selected topic
            connections = [x for x in self.bag.connections if x.topic == self.topic]
            self.msgs = self.bag.messages(connections=connections)
            self.previous_imu_timestamp = 0

        def __len__(self):
            return self.n_scans

        def __getitem__(self, idx):
            """
            DON'T call it directly. It is for iteration.
            """
            connection, _, rawdata = next(self.msgs)
            msg = self.bag.deserialize(rawdata, connection.msgtype)

            imu, imu_timestamp = self.read_imu(msg)
            self.prev_timestamp_head = self.timestamp_head
            self.timestamp_head = imu_timestamp
            frame_data = {"imu": imu, "timestamp": imu_timestamp}

            self.idx_ros_imu = self.idx_ros_imu + 1
            return frame_data

        def _get_first_timestamp_head(self):
            connections = [x for x in self.bag.connections if x.topic == self.topic]
            self.msgs = self.bag.messages(connections=connections)
            connection, _, rawdata = next(self.msgs)
            msg = self.bag.deserialize(rawdata, connection.msgtype)
            _, imu_timestamp = self.read_imu(msg)
            return imu_timestamp

        def load_data_to_txt(self, txtfile):
            file = open(txtfile, "w+")
            for frame_id in tqdm(range(self.n_scans)):
                file.write(str(round(self[frame_id]["timestamp"], 10)))
                file.write("\n")

            file.close()