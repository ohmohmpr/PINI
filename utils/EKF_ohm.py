import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

import copy

import LIOEKF_pybind
from utils.lio_para import LIO_Parameters
from utils.tools import transfrom_to_homo
import open3d as o3d
import os 

YELLOW = np.array([1, 0.706, 0])
RED = np.array([255, 0, 0]) / 255.0
PURPLE = np.array([238, 130, 238]) / 255.0
BLACK = np.array([0, 0, 0]) / 255.0
GOLDEN = np.array([1.0, 0.843, 0.0])
GREEN = np.array([0, 128, 0]) / 255.0
BLUE = np.array([0, 0, 128]) / 255.0
LIGHTBLUE = np.array([0.00, 0.65, 0.93])

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

class EKF_ohm:
    def __init__(self, config, LIOPara, o3d_vis, tracker,
                 dataset): #debug

        self.LIOEKF = LIOEKF_pybind._LIOEKF(LIOPara)
        self.LIOEKF._openResults()
        self.LIOEKF._init()
        self.LIOPara = LIOPara
        self.config = config
        self.config.sensor_fusion = self.config.sensor_fusion

        self.dataset = dataset # debug
        self.tracker = tracker

        # Visual
        self.o3d_vis = o3d_vis
        self.imu = o3d.geometry.TriangleMesh()

        #
        self.lidar_updated_ = self.LIOEKF.lidar_updated_
        self.pre_pose = np.identity(4)

        #
        self.pose_lidar_torch = None
        # if this thing work delete the belows.
        # if this thing work delete the belows.
        self.T_imu_NED = np.linalg.inv(transfrom_to_homo(LIOPara.imu_tran_R))
        self.T_imu_LiDAR = LIOPara.Trans_lidar_imu
        self.T_LiDAR_NED = np.linalg.inv(self.T_imu_LiDAR) @ self.T_imu_NED # please check

        self.config = config
        path_debug = os.path.join(self.config.run_path, "EKF_ohm.txt")
        self.debug_file = open(path_debug, "w+")

        path_delta_x = os.path.join(self.config.run_path, "delta_x_ohm.txt")
        self.delta_x_file = open(path_delta_x, "w+")
        np.printoptions(precision=10)

    def addLidarData(self, points, timestamp: list, point_ts: list):
        self.LIOEKF._addLidarData([LIOEKF_pybind._Vector3dVector(points)], [timestamp], [point_ts])

    def addImuData(self, imu: list, compensate: bool):
        self.LIOEKF._addImuData(imu, compensate)

    def convert_IMU(self, imu_ts, imu_dt, imu_la, imu_av):
        IMU = LIOEKF_pybind._IMU(imu_ts, imu_dt, imu_la, imu_av)
        return IMU
    
    def writeResults(self):
        self.LIOEKF._writeResults()

    def publishMsgs(self):
        self.LIOEKF._publishMsgs()

    def lidar_updated(self, boolean):
        self.lidar_updated_ = boolean
        self.LIOEKF.lidar_updated_ = boolean

    def update_map(self, neural_points):
        pose_in_lidar_frame = self.LIOPara.Trans_lidar_imu
        neural_points_numpy = neural_points.cpu().numpy()
        self.LIOEKF._lio_map_._update(LIOEKF_pybind._Vector3dVector(neural_points_numpy), pose_in_lidar_frame)

    # get
    def getKeyPoints(self):
        kw = self.LIOEKF._getKeyPoints_w()
        KeyPoints_w = np.asarray(kw)
        return KeyPoints_w
    
    def getCurrentPoints(self):
        pts = self.LIOEKF._getFrame_w()
        current_pts = np.asarray(pts)

        return current_pts

    def get_bodystate(self, Bodystate):
        '''
        self.LIOEKF._bodystate_cur_
        self.LIOEKF._bodystate_pre_
        '''
        pose_NED, vel_NED = self.LIOEKF._getBodyState(Bodystate)

        return pose_NED
    def get_bodystate_torch(self, Bodystate):
        pose_NED, vel_NED = self.LIOEKF._getBodyState(Bodystate)
        imu_tran_R = transfrom_to_homo(self.LIOPara.imu_tran_R)
        Trans_lidar_imu = self.LIOPara.Trans_lidar_imu
        pose_NED_torch = torch.tensor(np.linalg.inv(Trans_lidar_imu) @ pose_NED @ imu_tran_R, device=self.config.device, dtype=self.config.tran_dtype)

        return pose_NED_torch

    # main
    def newImuProcess_EKF(self):
        self.LIOEKF._newImuProcess()

    def newImuProcess_ohm(self, myupdate=False):

        if (self.LIOEKF._is_first_imu_):
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._imupre_ = self.LIOEKF._imucur_
            self.LIOEKF._imu_t_ = self.LIOEKF._imucur_.timestamp
            self.LIOEKF._is_first_imu_ = False
            return

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

            if (self.LIOEKF._is_first_lidar_): 
                pose_for_map = self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                if myupdate == False:
                    self.LIOEKF._lidarUpdate()
                else:
                    pose_for_map = self.update_EKF()

            self.lidar_updated(True)

            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        elif (lidarUpdateFlag == 2):
            # lidardata is near current imudata, we should firstly propagate navigation state
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)

            if self.LIOEKF._is_first_lidar_:
                pose_for_map = self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                if myupdate == False:
                    self.LIOEKF._lidarUpdate()
                else:
                    pose_for_map = self.update_EKF()

            self.lidar_updated(True)
        elif (lidarUpdateFlag == 3):
            # lidardata is between the two imudata, we interpolate current imudata to lidar time
            midimu = LIOEKF_pybind._IMU()
            LIOEKF_pybind._imuInterpolate(self.LIOEKF._imupre_, self.LIOEKF._imucur_, updatetime, midimu)

            # propagate navigation state for the first half imudata
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, midimu)

            # do lidar position update at the whole second and feedback system states
            if (self.LIOEKF._is_first_lidar_):
                pose_for_map = self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                if myupdate == False:
                    self.LIOEKF._lidarUpdate()
                else:
                    pose_for_map = self.update_EKF()
            self.lidar_updated(True)

            # propagate navigation state for the second half imudata
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(midimu, self.LIOEKF._imucur_)

        # check diagonal elements of current covariance matrix
        self.LIOEKF._checkStateCov()

        # update system state and imudata at the previous epoch
        self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        self.LIOEKF._imupre_ = self.LIOEKF._imucur_


    def newImuProcess_ohm_update(self):

        cur_pose_torch = None
        cur_odom_cov = None
        weight_pc_o3d = None
        valid_flag = None
        sdf_res = None
        J_mat = None
        if (self.LIOEKF._is_first_imu_):
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._imupre_ = self.LIOEKF._imucur_
            self.LIOEKF._imu_t_ = self.LIOEKF._imucur_.timestamp
            self.LIOEKF._is_first_imu_ = False

            return None, None, None, None, None, None

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

            if (self.LIOEKF._is_first_lidar_): 
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            # else:
                ###############################################################
                # deskew_points = self.LIOEKF._processScanPin()
                # deskew_points = np.asarray(deskew_points)
                # deskew_points_torch = torch.tensor(
                #     deskew_points, device=self.config.device, dtype=self.config.dtype
                # )
                # pred_pose_LiDAR_torch, pred_vel_LiDAR, _ = self.get_bodystate_for_prediction_torch(self.LIOEKF._bodystate_cur_)
                # tracking_result = self.tracker.tracking(deskew_points_torch, pred_pose_LiDAR_torch, 
                #                         self.dataset.cur_source_colors, dataset=self.dataset)
                # cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag, sdf_res, J_mat = tracking_result
    
                # self.update_EKF_SDF(sdf_res, J_mat)
                ###############################################################

            self.lidar_updated(True)

            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        elif (lidarUpdateFlag == 2):
            # lidardata is near current imudata, we should firstly propagate navigation state
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)

            if self.LIOEKF._is_first_lidar_:
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            # else:
                ###############################################################
                # deskew_points = self.LIOEKF._processScanPin()
                # deskew_points = np.asarray(deskew_points)
                # deskew_points_torch = torch.tensor(
                #     deskew_points, device=self.config.device, dtype=self.config.dtype
                # )
                # pred_pose_LiDAR_torch, pred_vel_LiDAR, _ = self.get_bodystate_for_prediction_torch(self.LIOEKF._bodystate_cur_)
                # tracking_result = self.tracker.tracking(deskew_points_torch, pred_pose_LiDAR_torch, 
                #                         self.dataset.cur_source_colors, dataset=self.dataset)
                # cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag, sdf_res, J_mat = tracking_result

                # self.update_EKF_SDF(sdf_res, J_mat)
                ###############################################################

            self.lidar_updated(True)
        elif (lidarUpdateFlag == 3):
            # lidardata is between the two imudata, we interpolate current imudata to lidar time
            midimu = LIOEKF_pybind._IMU()
            LIOEKF_pybind._imuInterpolate(self.LIOEKF._imupre_, self.LIOEKF._imucur_, updatetime, midimu)

            # propagate navigation state for the first half imudata
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, midimu)

            # do lidar position update at the whole second and feedback system states
            if (self.LIOEKF._is_first_lidar_):
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            # else:
                ###############################################################
                # deskew_points = self.LIOEKF._processScanPin()
                # deskew_points = np.asarray(deskew_points)
                # deskew_points_torch = torch.tensor(
                #     deskew_points, device=self.config.device, dtype=self.config.dtype
                # )
                # pred_pose_LiDAR_torch, pred_vel_LiDAR, _ = self.get_bodystate_for_prediction_torch(self.LIOEKF._bodystate_cur_)
                # tracking_result = self.tracker.tracking(deskew_points_torch, pred_pose_LiDAR_torch, 
                #                         self.dataset.cur_source_colors, dataset=self.dataset)
                # cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag, sdf_res, J_mat = tracking_result
                
                # self.update_EKF_SDF(sdf_res, J_mat)
                ###############################################################

            self.lidar_updated(True)

            # propagate navigation state for the second half imudata
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(midimu, self.LIOEKF._imucur_)

        # check diagonal elements of current covariance matrix
        self.LIOEKF._checkStateCov()

        # update system state and imudata at the previous epoch
        self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        self.LIOEKF._imupre_ = self.LIOEKF._imucur_

        if lidarUpdateFlag == 1:
            # print("lidarUpdateFlag", lidarUpdateFlag)
            cur_pose_torch, _, _ = self.get_bodystate_for_prediction_torch(self.LIOEKF._bodystate_cur_)
            return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag, sdf_res, J_mat
        elif lidarUpdateFlag == 2:
            # print("lidarUpdateFlag", lidarUpdateFlag)
            cur_pose_torch, _, _ = self.get_bodystate_for_prediction_torch(self.LIOEKF._bodystate_cur_)
            return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag, sdf_res, J_mat
        elif lidarUpdateFlag == 3:
            # print("lidarUpdateFlag", lidarUpdateFlag)
            cur_pose_torch, _, _ = self.get_bodystate_for_prediction_torch(self.LIOEKF._bodystate_cur_)
            return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag, sdf_res, J_mat
        elif lidarUpdateFlag == 0:
            return None, None, None, None, None, None
        return 

    # def update(self, residual: torch.tensor, jacobian: torch.tensor):
    #     # check if residual and jacobian have same rows
    #     assert residual.shape[0] == jacobian.shape[0]

    #     # print("residual : ", )
    #     POS_ID = 0
    #     VEL_ID = 3
    #     ATT_ID = 6
    #     H = torch.zeros(jacobian.shape[0], 15, device=self.config.device, dtype=self.config.tran_dtype)
    #     H[:, POS_ID:POS_ID+3] = jacobian[:, 3:6]
    #     H[:, ATT_ID:ATT_ID+3] = jacobian[:, 0:3]

    #     R = torch.eye(residual.shape[0], device=self.config.device, dtype=self.config.tran_dtype) * 1000

    #     state_cov_torch = torch.tensor(self.LIOEKF._Cov_, device=self.config.device, dtype=self.config.tran_dtype)

    #     HTRH = H.T @ R @ H
    #     HTRz = H.T @ R @ residual.to(dtype=torch.float64)

    #     S_inv = torch.inverse(HTRH + torch.inverse(state_cov_torch))
    #     delta_x_torch = S_inv @ HTRz
    #     state_cov_torch = state_cov_torch - S_inv @ HTRH @ state_cov_torch
    #     self.LIOEKF._delta_x_ = delta_x_torch.cpu().numpy()
    #     self.LIOEKF._Cov_ = state_cov_torch.cpu().numpy()

    #     # print(f"delta_x: {self.LIOEKF._delta_x_}")

    #     # self.stateFeedback()
    #     self.LIOEKF._stateFeedback()
    #     # self.delta_x = np.zeros((15, 1))
    #     self.LIOEKF._delta_x_ = np.zeros((15, 1))

    #     # self.prebodystate = copy.copy(self.curbodystate)
    #     self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_

    #     # # return lidar pose and cov
    #     # return torch.tensor(pos_T_LiDAR, device=config.device, dtype=config.tran_dtype)


    def so3_hat(self, omega: torch.Tensor) -> torch.Tensor:
        """
        skew-symmetric matrix or so(3) hat operator.
        """
        if omega.shape[-1] != 3:
            raise ValueError("Input must be a 3D vector or batch of 3D vectors.")

        x, y, z = omega[..., 0], omega[..., 1], omega[..., 2]
        
        hat = torch.zeros(omega.shape[:-1] + (3, 3), dtype=self.config.tran_dtype, device=self.config.device)
        hat[..., 0, 1] = -z
        hat[..., 0, 2] = y
        hat[..., 1, 0] = z
        hat[..., 1, 2] = -x
        hat[..., 2, 0] = -y
        hat[..., 2, 1] = x
        
        return hat

    def update_EKF(self):
        source, frame_downsample = self.LIOEKF._processScan()
        imu_pose_covariance = np.identity(6)
        imu_pose_covariance[0:3, 0:3] = self.LIOEKF._Imu_Prediction_Covariance_[0:3, 0:3]
        imu_pose_covariance[3:6, 3:6] = self.LIOEKF._Imu_Prediction_Covariance_[6:9, 6:9]
        imu_pose_covariance[0:3, 3:6] = self.LIOEKF._Imu_Prediction_Covariance_[0:3, 6:9]
        imu_pose_covariance[3:6, 0:3] = self.LIOEKF._Imu_Prediction_Covariance_[6:9, 0:3]

        pose_NED_cur = self.get_bodystate(self.LIOEKF._bodystate_cur_)
        pose_NED_pre = self.get_bodystate(self.LIOEKF._bodystate_pre_)
        relative_pose = np.linalg.inv(pose_NED_pre) @ pose_NED_cur

        uncertainty_motion = LIOEKF_pybind._propagateUscendentEigen(relative_pose, imu_pose_covariance)
        map_uncertainty = np.square(self.LIOPara.voxel_size / np.sqrt(self.LIOPara.max_points_per_voxel))
        range_uncertainty = np.square(0.05)
        max_correspondence_distance = 6 * np.sqrt(uncertainty_motion + map_uncertainty + range_uncertainty)

        last_dx = torch.zeros(1, 15, device=self.config.device, dtype=self.config.tran_dtype)
        weight = 1000
        j = 0
        cur_pose = pose_NED_cur

        for j in range(self.LIOPara.max_iteration):
            points_w = source

            self.LIOEKF._TransformPoints(cur_pose, points_w)

            src, tgt = self.LIOEKF._lio_map_._get_correspondences(points_w, max_correspondence_distance)
            src_numpy = np.asarray(src)
            tgt_numpy = np.asarray(tgt)

            residual = src_numpy - tgt_numpy
            residual_torch = torch.tensor(residual, device=self.config.device, dtype=self.config.tran_dtype)
            residual_torch = torch.reshape(residual_torch, (residual_torch.shape[0], residual_torch.shape[1], 1))
            R_bG_p = src_numpy - cur_pose[:3, 3]
            R_bG_p_torch = torch.tensor(R_bG_p, device=self.config.device, dtype=self.config.tran_dtype)

            H = torch.zeros((R_bG_p_torch.shape[0], 3, 15), device=self.config.device, dtype=self.config.tran_dtype)
            H[:, :, 0:3] = torch.eye(3)
            R_bG_p_torch_so3_hat = self.so3_hat(R_bG_p_torch)
            H[:, :, 6:9] = R_bG_p_torch_so3_hat

            R_inv = torch.zeros([residual_torch.shape[0], 3, 3], device=self.config.device, dtype=self.config.tran_dtype)
            R_inv[:, 0:3, 0:3] = torch.eye(3) * weight
            H_T = torch.transpose(H, 1, 2)
            HTRH = torch.matmul(H_T, torch.matmul(R_inv, H))
            HTRz = torch.matmul(H_T, torch.matmul(R_inv, residual_torch))

            HTRH = torch.sum(HTRH, 0)
            HTRz = torch.sum(HTRz, 0)

            state_cov_torch = torch.tensor(self.LIOEKF._Cov_, device=self.config.device, dtype=self.config.tran_dtype)
            S_inv = torch.inverse(HTRH + torch.inverse(state_cov_torch))

            delta_x_torch = S_inv @ HTRz
            self.LIOEKF._delta_x_ = delta_x_torch.cpu().numpy()
            KH = S_inv @ HTRH

            self.LIOEKF._stateFeedback()

            if ( torch.norm(delta_x_torch - last_dx) < 0.001):
                continue
            last_dx = delta_x_torch
            self.LIOEKF._delta_x_ = np.zeros(15)

        state_cov_torch -= KH @ state_cov_torch
        self.LIOEKF._Cov_ = state_cov_torch.cpu().numpy()

        cur_pose = self.get_bodystate(self.LIOEKF._bodystate_cur_)
        pose_in_lidar_frame = cur_pose @ np.array(self.LIOPara.Trans_lidar_imu)

        self.LIOEKF._Imu_Prediction_Covariance_ = np.zeros((15, 15))
        self.LIOEKF._lio_map_._update(frame_downsample, pose_in_lidar_frame)
        self.LIOEKF._last_update_t_ = self.LIOEKF._lidar_t_
        return pose_in_lidar_frame

    # def update_EKF_SDF(self, residual: torch.tensor, jacobian: torch.tensor):
    #     last_dx = torch.zeros(1, 15, device=self.config.device, dtype=self.config.tran_dtype)
    #     weight = 1000
    #     j = 0

    #     # torch.Size([1531]) # residual
        
    #     # torch.Size([1531, 6]) # jacobian

    #     max_num = self.LIOPara.max_iteration
    #     for j in range(max_num):

    #         residual_torch = torch.reshape(residual, (residual.shape[0], 1, 1)) # num, 3, 1
    #         # print("residual_torch", residual_torch)
    #         scale = 1
    #         residual_torch = torch.tensor(residual_torch.clone().detach(), device=self.config.device, dtype=self.config.tran_dtype) * scale
    #         jacobian_torch = torch.reshape(jacobian, (jacobian.shape[0], 1, jacobian.shape[1])) * scale # num, 3, 1
    #         # print("residual_torch", residual_torch.shape)
    #         # print("jacobian", jacobian_torch.shape)

    #         # R_bG_p = src_numpy - cur_pose[:3, 3]
    #         # R_bG_p_torch = torch.tensor(R_bG_p, device=self.config.device, dtype=self.config.tran_dtype)

    #         H = torch.zeros((jacobian.shape[0], 1, 15), device=self.config.device, dtype=self.config.tran_dtype) # num, 3, 15
    #         H[:, :, 0:3] = jacobian_torch[:, :, 3:6]
    #         # H[:, :, 0:3] = torch.ones(1, 3)
    #         # R_bG_p_torch_so3_hat = self.so3_hat(R_bG_p_torch)
    #         H[:, :, 6:9] = jacobian_torch[:, :, 0:3]

    #         R_inv = torch.ones([residual_torch.shape[0], 1, 1], device=self.config.device, dtype=self.config.tran_dtype) # num, 3, 3
    #         R_inv = R_inv * weight
    #         # R_inv[:, 0:3] = torch.ones(3) * weight
    #         H_T = torch.transpose(H, 1, 2)
    #         HTRH = torch.matmul(H_T, torch.matmul(R_inv, H))              # num,15,3 @ num,3,3 @ num,3,15
    #         HTRz = torch.matmul(H_T, torch.matmul(R_inv, residual_torch)) # num,15,3 @ num,3,3 @ num,3,1

    #         HTRH = torch.sum(HTRH, 0) # 15,3 @ 3,3 @ 3,15
    #         HTRz = torch.sum(HTRz, 0) # 15,3 @ 3,3 @ 3,1

    #         state_cov_torch = torch.tensor(self.LIOEKF._Cov_, device=self.config.device, dtype=self.config.tran_dtype)
    #         S_inv = torch.inverse(HTRH + torch.inverse(state_cov_torch))

    #         delta_x_torch = S_inv @ HTRz
    #         KH = S_inv @ HTRH
    #         print(delta_x_torch)

    #         self.LIOEKF._delta_x_ = delta_x_torch.cpu().numpy()
    #         print(self.LIOEKF._delta_x_)
    #         self.LIOEKF._stateFeedback()

    #         # if ((delta_x_ - last_dx).norm() < 0.001) {
    #         # break;
    #         # }
    #         # last_dx = delta_x_;
    #         self.LIOEKF._delta_x_ = np.zeros(15)


    #     state_cov_torch -= KH @ state_cov_torch
    #     self.LIOEKF._Cov_ = state_cov_torch.cpu().numpy()

    #     self.LIOEKF._last_update_t_ = self.LIOEKF._lidar_t_