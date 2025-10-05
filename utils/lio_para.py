import numpy as np
import LIOEKF_pybind

class LIO_Parameters:
    def __init__(self, config, topic):
        self.config = config.sensor_fusion
        self.topic = topic

    def init(self):
        D2R = (np.pi / 180.0)
        R2D = (180.0 / np.pi)
        NormG = 9.782940329221166

        self.config = self.config
        self.LIOPara = LIOEKF_pybind._LIOPara()
        self.LIOPara.deskew = self.config['main_sensor']['lidar']['deskew']
        self.LIOPara.preprocess = self.config['main_sensor']['lidar']['preprocess']
        self.LIOPara.max_range =  self.config['main_sensor']['lidar']['max_range']
        self.LIOPara.min_range =  self.config['main_sensor']['lidar']['min_range']
        self.LIOPara.max_points_per_voxel = self.config['main_sensor']['lidar']['max_points_per_voxel']

        self.LIOPara.voxel_size = self.config['main_sensor']['lidar']['voxel_size']
        self.LIOPara.max_iteration = self.config['main_sensor']['lidar']['max_iteration']

        self.LIOPara.initstate_std.pos = self.config['sensor_types']['imu'][0]['init_pos_std']
        self.LIOPara.initstate_std.vel = self.config['sensor_types']['imu'][0]['init_vel_std']
        self.LIOPara.initstate_std.euler = self.config['sensor_types']['imu'][0]['init_att_std']

        self.LIOPara.initstate_std.euler = self.LIOPara.initstate_std.euler * D2R

        self.LIOPara.imunoise.angle_randomwalk = np.array(self.config['sensor_types']['imu'][0]['arw']) * (D2R / 60.0)
        self.LIOPara.imunoise.velocity_randomwalk = np.array(self.config['sensor_types']['imu'][0]['vrw']) / 60.0
        self.LIOPara.imunoise.gyrbias_std = np.array(self.config['sensor_types']['imu'][0]['gbstd']) * (D2R / 3600.0)
        self.LIOPara.imunoise.accbias_std = np.array(self.config['sensor_types']['imu'][0]['abstd']) * 1e-5
        self.LIOPara.imunoise.correlation_time = np.array(self.config['sensor_types']['imu'][0]['corrtime']) * 3600

        self.LIOPara.initstate_std.imuerror.gyrbias = self.LIOPara.imunoise.gyrbias_std
        self.LIOPara.initstate_std.imuerror.accbias = self.LIOPara.imunoise.accbias_std


        self.LIOPara.topic = self.topic
        # depends on the topic
        for i in range(len(self.config['sensor_types']['imu'])):
            if self.topic == self.config['sensor_types']['imu'][i]["topic"]:
                # self.lidar = np.reshape(self.config['main_sensor']['lidar']['extrinsic_main_main'], (4, 4))
                self.imu_tran_R = np.reshape(self.config['sensor_types']['imu'][i]['imu_tran_R'], (3, 3))

                extrinsic_main_imu = np.reshape(
                    self.config['sensor_types']['imu'][i]['extrinsic_main_imu'], (4, 4))
                print("IMU TOPIC", self.config['sensor_types']['imu'][i]["topic"])

        extrinsic_R = extrinsic_main_imu[:3, :3]
        extrinsic_T = extrinsic_main_imu[:3, 3]

        Trans_lidar_imu_origin = np.identity(4)
        Trans_lidar_imu_origin[:3, :3] = extrinsic_R
        Trans_lidar_imu_origin[:3, 3] = extrinsic_T
        self.LIOPara.Trans_lidar_imu_origin = Trans_lidar_imu_origin

        extrinsic_R = self.imu_tran_R @ extrinsic_R
        extrinsic_T = self.imu_tran_R @ extrinsic_T
        self.LIOPara.imu_tran_R = self.imu_tran_R

        Trans_lidar_imu = np.identity(4)
        Trans_lidar_imu[:3, :3] = extrinsic_R
        Trans_lidar_imu[:3, 3] = extrinsic_T
        self.LIOPara.Trans_lidar_imu = Trans_lidar_imu

        return self.LIOPara


