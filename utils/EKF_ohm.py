import numpy as np
import torch
from scipy.spatial.transform import Rotation as R
import copy

import LIOEKF_pybind
from utils.lio_para import LIO_Parameters
from utils.tools import transfrom_to_homo
import sophuspy as sp

class StateID():
    POS_ID = 0
    VEL_ID = 3
    ATT_ID = 6
    GYRO_BIAS_ID = 9
    ACC_BIAS_ID = 12

class NoiseID():
    VEL_RANDOMWALK_ID = 0
    ANGEL_RANDOMWALK_ID = 3
    GYRO_BIAS_STD_ID = 6
    ACC_BIAS_STD_ID = 9

class IMU:
    def __init__(self, gyro=None, acc=None, timestamp=None, dt=None):
        self.gyro = gyro
        self.acc = acc
        self.timestamp = timestamp
        self.dt = dt

class IMUError:
    def __init__(self, gyro_bias=None, acc_bias=None):
        if gyro_bias is None:
            self.gyro_bias = np.zeros(3)
        else:
            self.gyro_bias = gyro_bias
        if acc_bias is None:
            self.acc_bias = np.zeros(3)
        else:
            self.acc_bias = acc_bias


class BodyState:
    def __init__(self, pose=None, vel=None):
        if pose is None:
            # Default pose: identity rotation and zero translation
            self.pose = np.eye(4)
        else:
            self.pose = pose
            
        if vel is None:
            # Default velocity: zero vector
            self.vel = np.zeros(3)
        else:
            self.vel = vel

def skew_symmetric(vec):
    return np.array([[0, -vec[2], vec[1]], [vec[2], 0, -vec[0]], [-vec[1], vec[0], 0]])

def create_covariance_block(vec):
    return np.diag(np.abs(vec)**2)
class EKF_ohm:
    def __init__(self, LIOPara):

        self.LIOEKF = LIOEKF_pybind._LIOEKF(LIOPara)
        self.LIOEKF._openResults()
        self.LIOEKF._init()

        self.lidar_updated_ = self.LIOEKF.lidar_updated_
        self.pre_pose = np.identity(4)

        self.pose_lidar_torch = None
        self.T_NED_imu = transfrom_to_homo(LIOPara.imu_tran_R)
        self.T_imu_LiDAR = np.linalg.inv(LIOPara.ext_imu_main)
        self.T_NED_LiDAR = self.T_NED_imu # self.T_imu_LiDAR @ 
        self.T_LiDAR_NED = np.linalg.inv(self.T_NED_LiDAR)

        # self.state_covariance = np.zeros((15, 15))
        # self.noise_covariance = np.zeros((12, 12))

        # # initialize 15 dimensional state vector
        # self.delta_x = np.zeros((15, 1))
        
        # self.prebodystate = BodyState()
        # self.curbodystate = BodyState()

        # self.imuerror = IMUError()

        # self.curimu = IMU()
        # self.preimu = IMU()

        # self.stateid = StateID()
        # self.noiseid = NoiseID()

        # self.imu_lidar_extrinsic = imu_lidar_extrinsic

        # # TODO
        # # Initialize state covariance matrix and noise covariance matrix
        # initposstd = np.array([0.01, 0.01, 0.01])
        # initvelstd = np.array([ 0.05, 0.05, 0.05 ])
        # initattstd = np.array([ 0.1, 0.1, 0.1 ]) * np.pi / 180
        # initgyrobiasstd = np.array([ 200.0, 200.0, 200.0 ]) * np.pi / 180 / 3600
        # initaccbiasstd = np.array([ 500.0, 500.0, 500.0 ]) * 1e-5
        # arw = np.array([2, 2, 2]) * np.pi / 180 / 60
        # vrw = np.array([40, 40, 40]) / 60
        # coortime = 1 * 3600

        # # set initial state covariance
        # self.state_covariance[self.stateid.POS_ID:self.stateid.POS_ID+3, self.stateid.POS_ID:self.stateid.POS_ID+3] = create_covariance_block(initposstd)
        # self.state_covariance[self.stateid.VEL_ID:self.stateid.VEL_ID+3, self.stateid.VEL_ID:self.stateid.VEL_ID+3] = create_covariance_block(initvelstd)
        # self.state_covariance[self.stateid.ATT_ID:self.stateid.ATT_ID+3, self.stateid.ATT_ID:self.stateid.ATT_ID+3] = create_covariance_block(initattstd)
        # self.state_covariance[self.stateid.GYRO_BIAS_ID:self.stateid.GYRO_BIAS_ID+3, self.stateid.GYRO_BIAS_ID:self.stateid.GYRO_BIAS_ID+3] = create_covariance_block(initgyrobiasstd)
        # self.state_covariance[self.stateid.ACC_BIAS_ID:self.stateid.ACC_BIAS_ID+3, self.stateid.ACC_BIAS_ID:self.stateid.ACC_BIAS_ID+3] = create_covariance_block(initaccbiasstd)

        # # set noise covariance
        # self.noise_covariance[self.noiseid.VEL_RANDOMWALK_ID:self.noiseid.VEL_RANDOMWALK_ID+3, self.noiseid.VEL_RANDOMWALK_ID:self.noiseid.VEL_RANDOMWALK_ID+3] = create_covariance_block(vrw)
        # self.noise_covariance[self.noiseid.ANGEL_RANDOMWALK_ID:self.noiseid.ANGEL_RANDOMWALK_ID+3, self.noiseid.ANGEL_RANDOMWALK_ID:self.noiseid.ANGEL_RANDOMWALK_ID+3] = create_covariance_block(arw)
        # self.noise_covariance[self.noiseid.GYRO_BIAS_STD_ID:self.noiseid.GYRO_BIAS_STD_ID+3, self.noiseid.GYRO_BIAS_STD_ID:self.noiseid.GYRO_BIAS_STD_ID+3] = create_covariance_block(initgyrobiasstd) * 2 / coortime
        # self.noise_covariance[self.noiseid.ACC_BIAS_STD_ID:self.noiseid.ACC_BIAS_STD_ID+3, self.noiseid.ACC_BIAS_STD_ID:self.noiseid.ACC_BIAS_STD_ID+3] = create_covariance_block(initaccbiasstd) * 2 / coortime

    def addLidarData(self, points, timestamp: list, point_ts: list):
        self.LIOEKF._addLidarData([LIOEKF_pybind._Vector3dVector(points)], [timestamp], [point_ts])

    def addImuData(self, imu: list, compensate: bool):
        self.LIOEKF._addImuData(imu, compensate)

    def convert_IMU(self, imu_ts, imu_dt, imu_la, imu_av):
        IMU = LIOEKF_pybind._IMU(imu_ts, 
                                imu_dt, 
                                imu_la, 
                                imu_av)
        return IMU
    
    def writeResults(self):
        self.LIOEKF._writeResults()

    def publishMsgs(self):
        self.LIOEKF._publishMsgs()

    # def set_bodystate_pre(self, bodystate_pre):
    #     self.LIOEKF._bodystate_pre_ = 

    def lidar_updated(self, boolean):
        self.lidar_updated_ = boolean
        self.LIOEKF.lidar_updated_(boolean)

    # def processIMU(self, frame_imu_data):
    #     # iterate frame_imu_data
    #     for i in range(len(frame_imu_data['ts'])):
            
    #         tmp_imu = self.convertIMU({key: value[i] for key, value in frame_imu_data.items()})
    #         if tmp_imu.timestamp <= self.curimu.timestamp:
    #             self.preimu = self.curimu
    #             continue
    #         self.curimu = tmp_imu
    #         self.imuCompensate()
    #         # print(f"preimu ts: {self.preimu.timestamp} gyro: {self.preimu.gyro}, acc: {self.preimu.acc}, dt: {self.preimu.dt}")
    #         # print(f"curimu ts: {self.curimu.timestamp} gyro: {self.curimu.gyro}, acc: {self.curimu.acc}, dt: {self.curimu.dt}")

    #         self.statePropagation()
    #         self.prebodystate = copy.copy(self.curbodystate)
    #         self.preimu = self.curimu
             
    #         lidarpose = self.getcurLidarpose()
    #         lidar_att = R.from_matrix(lidarpose[:3,:3]).as_euler('xyz')*180/np.pi
    #         lidar_pos = lidarpose[:3,3]
    #         imu_att = R.from_matrix(self.curbodystate.pose[:3,:3]).as_euler('xyz')*180/np.pi
    #         imu_pos = self.curbodystate.pose[:3,3]
    #         # print(f"ts: {self.curimu.timestamp} lidar pos: {lidar_pos}, lidar att: {lidar_att}")
    #         # print(f"ts: {self.curimu.timestamp} imu: {self.curimu.gyro} {self.curimu.acc} dt: {self.curimu.dt}")
    #         print(f"ts: {self.curimu.timestamp} imu   pos: {imu_pos}, imu   att: {imu_att} imu vel: {self.curbodystate.vel}")

    #         # print the diagonal of state covariance
    #         print(f"state covariance: {np.diag(self.state_covariance)}")

    def newImuProcess(self, dataset, tracker, config, topic):

        cur_pose_torch = None
        cur_odom_cov = None 
        weight_pc_o3d = None
        valid_flag = None
        if (self.LIOEKF._is_first_imu_):

            last_pose_T_NED = self.T_NED_LiDAR @ dataset.last_pose_ref
            last_vel_T_NED = self.T_NED_LiDAR[:3, :3] @ dataset.last_velocity
            self.LIOEKF._setBodyStateCurrent(last_pose_T_NED, last_vel_T_NED)
            state = self.LIOEKF._getNavState()

            # print("self.LIOEKF._is_first_imu_\n", self.LIOEKF._is_first_imu_)
            # print("last_pose_T_NED\n", last_pose_T_NED)
            # print("state.pos\n", state.pos)
            # print("state.vel\n", state.vel)
            # print("self.LIOEKF._bodystate_pre_.vel", self.LIOEKF._bodystate_pre_.vel)
            # print("self.LIOEKF._bodystate_cur_.pose", self.LIOEKF._bodystate_cur_.pose)
            # print("self.LIOEKF._bodystate_cur_.vel", self.LIOEKF._bodystate_cur_.vel)
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._imupre_ = self.LIOEKF._imucur_
            self.LIOEKF._imu_t_ = self.LIOEKF._imucur_.timestamp
            self.LIOEKF._is_first_imu_ = False
            return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag

        # set update time as the lidar time stamp
        # double
        updatetime = self.LIOEKF._lidar_t_

        lidarUpdateFlag = 0

        if (self.LIOEKF._lidar_t_ > self.LIOEKF._last_update_t_ + 0.001 or self.LIOEKF._is_first_lidar_):
            lidarUpdateFlag = self.LIOEKF._isToUpdate(self.LIOEKF._imupre_.timestamp, self.LIOEKF._imucur_.timestamp, updatetime)


        if (lidarUpdateFlag == 0):
            # only propagate navigation state
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        elif (lidarUpdateFlag == 1):
            #  lidardata is near to the previous imudata, we should firstly do lidar
            #  update

            last_pose_T_NED = self.T_NED_LiDAR @ dataset.last_pose_ref
            last_vel_T_NED = self.T_NED_LiDAR[:3, :3] @ dataset.last_velocity
            self.LIOEKF._setBodyStateCurrent(last_pose_T_NED, last_vel_T_NED)
            state = self.LIOEKF._getNavState()

            # print("self.LIOEKF._is_first_imu_\n", self.LIOEKF._is_first_imu_)
            # print("last_pose_T_NED\n", last_pose_T_NED)
            # print("state.pos\n", state.pos)
            # print("state.vel\n", state.vel)
            if (self.LIOEKF._is_first_lidar_): 
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
                ###########################################################################################
                cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
                ###########################################################################################
                # self.LIOEKF._lidarUpdate()
            else:
                ###########################################################################################
                cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
                ###########################################################################################
                # self.LIOEKF._lidarUpdate()
            
            self.LIOEKF.lidar_updated_ = True

            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        elif (lidarUpdateFlag == 2):
            # lidardata is near current imudata, we should firstly propagate navigation state
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)


            last_pose_T_NED = self.T_NED_LiDAR @ dataset.last_pose_ref
            last_vel_T_NED = self.T_NED_LiDAR[:3, :3] @ dataset.last_velocity
            self.LIOEKF._setBodyStateCurrent(last_pose_T_NED, last_vel_T_NED)
            state = self.LIOEKF._getNavState()

            # print("self.LIOEKF._is_first_imu_\n", self.LIOEKF._is_first_imu_)
            # print("last_pose_T_NED\n", last_pose_T_NED)
            # print("state.pos\n", state.pos)
            # print("state.vel\n", state.vel)
            if self.LIOEKF._is_first_lidar_:
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
                ###########################################################################################
                cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
                ###########################################################################################
                # self.LIOEKF._lidarUpdate()
            else:
                ###########################################################################################
                cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
                ##########################################################################################
                # self.LIOEKF._lidarUpdate()

            self.LIOEKF.lidar_updated_ = True
        elif (lidarUpdateFlag == 3):
            # lidardata is between the two imudata, we interpolate current imudata to lidar time
            midimu = LIOEKF_pybind._IMU()
            LIOEKF_pybind._imuInterpolate(self.LIOEKF._imupre_, self.LIOEKF._imucur_, updatetime, midimu)

            # propagate navigation state for the first half imudata
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, midimu)


            last_pose_T_NED = self.T_NED_LiDAR @ dataset.last_pose_ref
            last_vel_T_NED = self.T_NED_LiDAR[:3, :3] @ dataset.last_velocity
            self.LIOEKF._setBodyStateCurrent(last_pose_T_NED, last_vel_T_NED)
            state = self.LIOEKF._getNavState()

            # print("self.LIOEKF._is_first_imu_\n", self.LIOEKF._is_first_imu_)
            # print("last_pose_T_NED\n", last_pose_T_NED)
            # print("state.pos\n", state.pos)
            # print("state.vel\n", state.vel)
            # do lidar position update at the whole second and feedback system states
            if (self.LIOEKF._is_first_lidar_):
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
                ###########################################################################################
                cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
                ###########################################################################################
                # self.LIOEKF._lidarUpdate()
            else:
                ###########################################################################################
                cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
                ###########################################################################################
                # self.LIOEKF._lidarUpdate()
            self.LIOEKF.lidar_updated_ = True

            # propagate navigation state for the second half imudata
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(midimu, self.LIOEKF._imucur_)

        # check diagonal elements of current covariance matrix
        self.LIOEKF._checkStateCov()

        # update system state and imudata at the previous epoch
        self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        self.LIOEKF._imupre_ = self.LIOEKF._imucur_
        return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag
    # def getcurLidarpose(self) -> np.array:
    #     lidarpose = np.linalg.inv(self.trans_wi_wl) @ self.curbodystate.pose @ self.imu_lidar_extrinsic
    #     return lidarpose

    # TODO torch parallelization for update


    def update_ohm(self, dataset, tracker, config, topic):
        
        state = self.LIOEKF._getNavState_pin()
        pos_T_NED = np.zeros((4, 4))
        pos_T_NED[:3, :3] = state.rot
        pos_T_NED[:3, 3] = state.pos
        pos_T_NED[3, 3] = 1

        # print("pos_T_NED\n", pos_T_NED)
        pos_T_LiDAR = self.T_LiDAR_NED @ pos_T_NED
        # print("pos_T_LiDAR\n", pos_T_LiDAR)

        pos_T_predict_LiDAR = pos_T_LiDAR

        pos_T_predict_LiDAR_torch = torch.tensor(pos_T_predict_LiDAR, device=config.device, dtype=config.tran_dtype)

        last_pose_ref = torch.tensor(
                dataset.last_pose_ref, dtype=torch.float64, device=config.device
        )   
        # print("pos_T_predict_LiDAR_torch\n", pos_T_predict_LiDAR_torch)
        if self.pose_lidar_torch == None:
            # print("self.pose_lidar_torch == None\n", dataset.cur_pose_guess_torch)
            self.pose_lidar_torch = dataset.cur_pose_guess_torch
        else:
            self.pose_lidar_torch = pos_T_predict_LiDAR_torch
        tracking_result = tracker.tracking(dataset.cur_source_points, self.pose_lidar_torch, 
                                        dataset.cur_source_colors)
        cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = tracking_result

        cur_pose = cur_pose_torch.cpu().numpy()

        # cur_pose_T_NED = self.T_NED_LiDAR @ cur_pose
        last_vel_T_NED = self.LIOEKF._bodystate_cur_.vel
        # print("cur_pose_T_NED", cur_pose_T_NED)
        # print("last_vel_T_LiDAR", self.T_LiDAR_NED[:3, :3] @ last_vel_T_NED)
        # self.LIOEKF._setBodyStateCurrent(cur_pose_T_NED, last_vel_T_NED) 

        return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag


    def update(self, residual: torch.tensor, jacobian: torch.tensor, config):
        # check if residual and jacobian have same rows
        assert residual.shape[0] == jacobian.shape[0]

        H = torch.zeros(jacobian.shape[0], 15, device=config.device, dtype=config.tran_dtype)
        H[:, self.stateid.POS_ID:self.stateid.POS_ID+3] = jacobian[:, 3:6]
        H[:, self.stateid.ATT_ID:self.stateid.ATT_ID+3] = jacobian[:, 0:3]

        R = torch.eye(residual.shape[0], device=config.device, dtype=config.tran_dtype) * 1000

        state_cov_torch = torch.tensor(self.state_covariance, device=config.device, dtype=config.tran_dtype)

        HTRH = H.T @ R @ H
        HTRz = H.T @ R @ residual.to(dtype=torch.float64)

        S_inv = torch.inverse(HTRH + torch.inverse(state_cov_torch))
        delta_x_torch = S_inv @ HTRz
        state_cov_torch = state_cov_torch - S_inv @ HTRH @ state_cov_torch
        self.delta_x = delta_x_torch.cpu().numpy()
        self.state_covariance = state_cov_torch.cpu().numpy()

        print(f"delta_x: {self.delta_x}")

        self.stateFeedback()
        self.delta_x = np.zeros((15, 1))

        self.prebodystate = copy.copy(self.curbodystate)
        # return lidar pose and cov
        return self.getcurLidarpose()

