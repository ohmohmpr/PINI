import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

import sophuspy as sp

from rich import print

import LIOEKF_pybind
from utils.tools import transfrom_to_homo
from utils.tracker import expmap
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
    def __init__(self, config, LIOPara, tracker,
                 dataset, neural_points, o3d_vis=None): #debug

        self.LIOEKF = LIOEKF_pybind._LIOEKF(LIOPara)
        self.LIOEKF._openResults()
        self.LIOEKF._init()
        self.LIOPara = LIOPara
        self.config = config
        self.config.sensor_fusion = self.config.sensor_fusion

        self.dataset = dataset # debug
        self.neural_points = neural_points
        self.tracker = tracker

        # Visual
        self.o3d_vis = o3d_vis
        self.imu = o3d.geometry.TriangleMesh()

        #
        self.lidar_updated_ = self.LIOEKF.lidar_updated_
        self.pre_pose = np.identity(4)
        self.IMU_orientation = np.identity(4)

        #
        self.pose_lidar_torch = None
        # if this thing work delete the belows.
        # if this thing work delete the belows.
        self.T_imu_NED = np.linalg.inv(transfrom_to_homo(LIOPara.imu_tran_R))
        self.T_imu_LiDAR = LIOPara.Trans_lidar_imu
        self.Trans_lidar_imu = self.LIOPara.Trans_lidar_imu
        self.T_LiDAR_NED = np.linalg.inv(self.T_imu_LiDAR) @ self.T_imu_NED # please check
        self.prev_state = np.identity(4)
        self.diff = np.identity(4)
        self.point_diff = np.identity(4)

        # files
        self.config = config
        path_delta_x = os.path.join(self.config.run_path, "delta_x_ohm.txt")
        path_state = os.path.join(self.config.run_path, "state.txt")

        self.delta_x_file = open(path_delta_x, "w+")
        self.state_file = open(path_state, "w+")

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
    def set_bodystate_LiDAR_to_IMU(self, pose_fLiDAR):
        '''
        get pose 4x4(6DoF) from LiDAR frame
        transform and set to IMU frame
        NED_IMU = Trans_lidar_imu + pose_fLiDAR
        Trans_lidar_imu = imu_tran_R @ Trans_lidar_imu_origin
        '''
        # self.LIOEKF._setPoseBodyStateCurrent(dataset.last_pose_ref)
        Trans_lidar_imu = self.LIOPara.Trans_lidar_imu
        imu_tran_R = transfrom_to_homo(self.LIOPara.imu_tran_R)
        pose_NED = Trans_lidar_imu @ pose_fLiDAR @ self.IMU_orientation @ imu_tran_R
        self.LIOEKF._setPoseBodyStateCurrent(pose_NED)

    def get_bodystate_fLiDAR_torch(self, Bodystate):
        pose_NED, vel_NED = self.LIOEKF._getBodyState(Bodystate)
        imu_tran_R = transfrom_to_homo(self.LIOPara.imu_tran_R)
        Trans_lidar_imu = self.LIOPara.Trans_lidar_imu
        pose_NED_torch = torch.tensor(np.linalg.inv(Trans_lidar_imu) @ pose_NED @ Trans_lidar_imu, device=self.config.device, dtype=self.config.tran_dtype)
        # pose_NED_torch = torch.tensor(np.linalg.inv(Trans_lidar_imu) @ pose_NED @ np.linalg.inv(self.IMU_orientation) @ imu_tran_R, device=self.config.device, dtype=self.config.tran_dtype)

        return pose_NED_torch

    # main
    def newImuProcess_EKF(self):
        self.LIOEKF._newImuProcess()

    def newImuProcess_ohm(self, myupdate=True):

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
            # ############# debug #############
            # delta_x_info = np.array([self.LIOEKF._imu_t_, self.dataset.frame_id, lidarUpdateFlag])
            # delta_x_info = np.reshape(delta_x_info, (1, delta_x_info.shape[0]))
            # delta_x = np.reshape(self.LIOEKF._delta_x_, (1, self.LIOEKF._delta_x_.shape[0]))
            # np.savetxt(self.state_file, np.hstack((delta_x_info, delta_x)), delimiter=',', fmt='%10.5f')
            ############# debug #############
        elif (lidarUpdateFlag == 1):
            #  lidardata is near to the previous imudata, we should firstly do lidar
            #  update

            if (self.LIOEKF._is_first_lidar_): 
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                if myupdate == False:
                    self.LIOEKF._lidarUpdate()
                else:
                    self.update_EKF()

            self.lidar_updated(True)

            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        elif (lidarUpdateFlag == 2):
            # lidardata is near current imudata, we should firstly propagate navigation state
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)

            if self.LIOEKF._is_first_lidar_:
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                if myupdate == False:
                    self.LIOEKF._lidarUpdate()
                else:
                    self.update_EKF()

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
            else:
                if myupdate == False:
                    self.LIOEKF._lidarUpdate()
                else:
                    self.update_EKF()
            self.lidar_updated(True)

            # propagate navigation state for the second half imudata
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(midimu, self.LIOEKF._imucur_)

        # check diagonal elements of current covariance matrix
        self.LIOEKF._checkStateCov()

        # update system state and imudata at the previous epoch
        self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        self.LIOEKF._imupre_ = self.LIOEKF._imucur_


    # def wrapper(self, dataset, tracker, topic, o3d_vis):
    def wrapper(self, dataset, tracker):

        self.newImuProcess_wrapper(dataset, tracker)
        # self.newImuProcess_wrapper(dataset, tracker, o3d_vis, int_imu)

    # def newImuProcess_wrapper(self, dataset, tracker, o3d_vis):
    def newImuProcess_wrapper(self, dataset, tracker):
        if (self.LIOEKF._is_first_imu_):

            # body_l = self.get_bodystate_fLiDAR_torch(self.LIOEKF._bodystate_cur_).cpu().numpy()
            # body_i = self.get_bodystate(self.LIOEKF._bodystate_cur_)
            # print("[bold green]FRAME ID: [/bold green]", dataset.frame_id)
            # print("[bold yellow](EKF_OHM)[/bold yellow](BEFORE assign): IMU in L frame,\n", 
            #     R.from_matrix(body_l[:3, :3]).as_euler('xyz', degrees=True),
            #     body_l[:3, 3])
            # print("[bold yellow](EKF_OHM)[/bold yellow](BEFORE assign): IMU in I frame,\n", 
            #     R.from_matrix(body_i[:3, :3]).as_euler('xyz', degrees=True),
            #     body_i[:3, 3])

            # print("\n[bold yellow](EKF_OHM)[/bold yellow](ASSIGN Orientation)", )
            # r = R.from_euler('xyz', [dataset.init_roll_degree, dataset.init_pitch_degree, 0], degrees=True)
            # print("[bold yellow](EKF_OHM)[/bold yellow](Orientation): as_matrix: \n", r.as_matrix())
            # print("[bold yellow](EKF_OHM)[/bold yellow](Orientation): as_euler(rpy): \n", r.as_euler('xyz', degrees=True))
            # self.IMU_orientation = transfrom_to_homo(r.as_matrix())
            # self.set_bodystate_LiDAR_to_IMU(body_l)

            # body_l = self.get_bodystate_fLiDAR_torch(self.LIOEKF._bodystate_cur_).cpu().numpy()
            # body_i = self.get_bodystate(self.LIOEKF._bodystate_cur_)

            # self.LIOPara.imunoise.gyrbias_std = np.deg2rad(dataset.init_gyro_bias_degree)

            # print("[bold yellow](EKF_OHM)[/bold yellow](assign Orientation): IMU in L frame,\n", 
            #     R.from_matrix(body_l[:3, :3]).as_euler('xyz', degrees=True),
            #     body_l[:3, 3])
            # print("[bold yellow](EKF_OHM)[/bold yellow](assign Orientation): IMU in I frame,\n", 
            #     R.from_matrix(body_i[:3, :3]).as_euler('xyz', degrees=True),
            #     body_i[:3, 3])
            # print("[bold red]These two terms should be diff by IMU Orientation + imu_tran + ext params[/bold red], ",
            #     R.from_matrix(body_l[:3, :3]).as_euler('xyz', degrees=True) - R.from_matrix(body_i[:3, :3]).as_euler('xyz', degrees=True),
            #     body_l[:3, 3] - body_i[:3, 3])
            # self.prev_state = body_i



            # print("\n [bold yellow]First IMU[/bold yellow]")

            # print("[bold blue](PIN_SLAM)[/bold blue] LiDAR state:\n", 
            #       R.from_matrix(dataset.last_pose_ref[:3, :3]).as_euler('xyz', degrees=True), 
            #       dataset.last_pose_ref[:3, 3])
            # print("[bold yellow](EKF_OHM)[/bold yellow] IMU drift :\n", 
            #       R.from_matrix(self.diff[:3, :3]).as_euler('xyz', degrees=True), 
            #       self.diff[:3, 3])
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._imupre_ = self.LIOEKF._imucur_
            self.LIOEKF._imu_t_ = self.LIOEKF._imucur_.timestamp
            self.LIOEKF._is_first_imu_ = False
            # o3d_vis.stop()
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
            # do lidar position update at the whole second and feedback system states
            if (self.LIOEKF._is_first_lidar_):
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                self.update_tracker(dataset, tracker)
            self.lidar_updated(True)

            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)

            body_i = self.get_bodystate(self.LIOEKF._bodystate_cur_)
            self.prev_state = body_i
            self.diff = np.identity(4)

        elif (lidarUpdateFlag == 2):
            # lidardata is near current imudata, we should firstly propagate navigation state
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
            
            # do lidar position update at the whole second and feedback system states
            if (self.LIOEKF._is_first_lidar_):
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                self.update_tracker(dataset, tracker)

            self.lidar_updated(True)

            body_i = self.get_bodystate(self.LIOEKF._bodystate_cur_)
            self.prev_state = body_i
            self.diff = np.identity(4)

        elif (lidarUpdateFlag == 3):
            # lidardata is between the two imudata, we interpolate current imudata to lidar time
            midimu = LIOEKF_pybind._IMU()
            LIOEKF_pybind._imuInterpolate(self.LIOEKF._imupre_, self.LIOEKF._imucur_, updatetime, midimu)

            # propagate navigation state for the first half imudata
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, midimu)

            # do lidar position update at the whole second and feedback system states
            if (self.LIOEKF._is_first_lidar_):
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                self.update_tracker(dataset, tracker)

            self.lidar_updated(True)

            # propagate navigation state for the second half imudata
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(midimu, self.LIOEKF._imucur_)

            body_i = self.get_bodystate(self.LIOEKF._bodystate_cur_)
            self.prev_state = body_i
            self.diff = np.identity(4)

        # check diagonal elements of current covariance matrix
        self.LIOEKF._checkStateCov()

        # update system state and imudata at the previous epoch
        self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        self.LIOEKF._imupre_ = self.LIOEKF._imucur_

    def update_tracker(self, dataset, tracker):

        # self.o3d_vis.vis.add_geometry(self.o3d_vis.keypoint_lio_ekf, self.o3d_vis.reset_bounding_box)
        initguess_torch = self.get_bodystate_fLiDAR_torch(self.LIOEKF._bodystate_cur_)

        deskew_points_torch = torch.tensor(
            np.asarray(self.LIOEKF._processScanPin()), device=self.config.device, dtype=self.config.dtype
        )
        # self.o3d_vis.keypoint_lio_ekf.points = o3d.utility.Vector3dVector(dataset.cur_source_points.cpu().numpy())
        # self.o3d_vis.keypoint_lio_ekf.paint_uniform_color(BLUE)
        # self.o3d_vis.vis.add_geometry(self.o3d_vis.keypoint_lio_ekf, self.o3d_vis.reset_bounding_box)
        # dataset.cur_source_points, dataset.cur_pose_guess_torch
        # deskew_points_torch, initguess_torch
        # dataset.cur_point_cloud_torch = deskew_points_torch
        dataset.cur_source_points = deskew_points_torch
        tracker.tracking(deskew_points_torch, initguess_torch, 
                                self.dataset.cur_source_colors, dataset=self.dataset, 
                                # o3d_vis=o3d_vis)
                                Trans_lidar_imu=self.Trans_lidar_imu,
                                imu_tran_R=transfrom_to_homo(self.LIOPara.imu_tran_R),
                                EKF_class=self,
                                EKF_update_PIN=self.update_EKF_PIN)

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
            self.point_diff = np.asarray(points_w).shape[0] - src_numpy.shape[0]
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

            ##########################################################################################
            point_diff = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, 11,
                               self.point_diff, np.zeros((14,))))
            point_diff = np.reshape(point_diff, (1, -1))
            np.savetxt(self.state_file, point_diff, delimiter=',', fmt='%10.5f')

            n_mat = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, 15,
                               torch.diagonal(HTRH).cpu().numpy()))
            n_mat = np.reshape(n_mat, (1, n_mat.shape[0]))
            np.savetxt(self.state_file, n_mat, delimiter=',', fmt='%10.5f')
    
            cov = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, 10,
                               torch.diagonal(state_cov_torch).cpu().numpy()))
            cov = np.reshape(cov, (1, cov.shape[0]))
            np.savetxt(self.state_file, cov, delimiter=',', fmt='%10.5f')
            state = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, -1,
                               self.LIOEKF._getNavState().pos, self.LIOEKF._getNavState().vel, self.LIOEKF._getNavState().euler,
                              self.LIOEKF._getNavState().imuerror.gyrbias, self.LIOEKF._getNavState().imuerror.accbias))
            state = np.reshape(state, (1, state.shape[0]))
            np.savetxt(self.state_file, state, delimiter=',', fmt='%10.5f')

            delta_x_info = np.array([self.LIOEKF._imu_t_, self.dataset.frame_id, 0])
            delta_x_info = np.reshape(delta_x_info, (1, delta_x_info.shape[0]))
            np.savetxt(self.state_file, np.hstack((delta_x_info, delta_x_torch.T.cpu().numpy())), delimiter=',', fmt='%10.5f')
            self.LIOEKF._stateFeedback()

            state = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, 1,
                               self.LIOEKF._getNavState().pos, self.LIOEKF._getNavState().vel, self.LIOEKF._getNavState().euler,
                              self.LIOEKF._getNavState().imuerror.gyrbias, self.LIOEKF._getNavState().imuerror.accbias))
            state = np.reshape(state, (1, state.shape[0]))
            np.savetxt(self.state_file, state, delimiter=',', fmt='%10.5f')
            ##########################################################################################

            if (torch.norm(delta_x_torch - last_dx) < 0.001):
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
        return delta_x_torch, pose_in_lidar_frame

    def update_EKF_PIN(self, 
            points,
            sdf_grad,
            sdf_residual,
            weight,
            sdf_total_res):

        last_dx = torch.zeros(1, 15, device=self.config.device, dtype=self.config.tran_dtype)

        for j in range(self.LIOPara.max_iteration):

            # flip into imu frame
            Trans_lidar_imu = self.LIOPara.Trans_lidar_imu

            points_np = np.hstack((points.cpu().numpy(), np.ones((points.shape[0], 1))))
            points_np_t = (Trans_lidar_imu @ points_np.T).T
            points = torch.tensor(points_np_t[:, :3], device=self.config.device, dtype=torch.float32)
            
            sdf_grad_np = np.hstack((sdf_grad.cpu().numpy(), np.ones((sdf_grad.shape[0], 1))))
            sdf_grad_np_t = (Trans_lidar_imu @ sdf_grad_np.T).T
            sdf_grad = torch.tensor(sdf_grad_np_t[:, :3], device=self.config.device, dtype=torch.float32)

            sdf_grad_r = R.from_rotvec(sdf_grad_np_t[:, :3])
            sdf_grad_rotation = torch.tensor(sdf_grad_r.as_quat()[:, :3], device=self.config.device, dtype=torch.float32)

            cross = torch.cross(points, sdf_grad_rotation, dim=-1)  # N,3 x N,3
            J_vel = torch.zeros_like(cross)
            J_gyro_bias = torch.zeros_like(cross)
            J_acc_bias = torch.zeros_like(cross)
            J_mat = torch.cat(
                [sdf_grad, J_vel, cross, J_gyro_bias, J_acc_bias], -1
            )  # The Jacobian matrix # first rotation, then translation # N, 6

            sdf_total_res = sdf_total_res.reshape(-1, 1)

            weight = 400 * 1/torch.sqrt(sdf_total_res) * weight
            N_mat = J_mat.T @ (
               weight * J_mat
            )  # approximate Hessian matrix # first tran, then rot # 6, 6

            g_vec = -(J_mat * weight).T @ sdf_residual

            state_cov_torch = torch.tensor(self.LIOEKF._Cov_, device=self.config.device, dtype=torch.float32)
            S_inv = torch.inverse(N_mat + torch.inverse(state_cov_torch))

            delta_x_torch = S_inv @ g_vec

            # Add factor, so that is compatible to LIO_EKF
            factor = -1
            # translation
            delta_x_torch[0] = factor * delta_x_torch[0]
            delta_x_torch[1] = factor * delta_x_torch[1]
            delta_x_torch[2] = factor * delta_x_torch[2]
            # velocity
            delta_x_torch[3] = factor * delta_x_torch[3]
            delta_x_torch[4] = factor * delta_x_torch[4]
            delta_x_torch[5] = factor * delta_x_torch[5]
            # acceleration bias
            delta_x_torch[12] = factor * delta_x_torch[12]
            delta_x_torch[13] = factor * delta_x_torch[13]
            delta_x_torch[14] = factor * delta_x_torch[14]
            ########################################################################
            self.LIOEKF._delta_x_ = delta_x_torch.cpu().numpy()
            KH = S_inv @ N_mat

            ##########################################################################################
            n_mat = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, 15,
                               torch.diagonal(N_mat).cpu().numpy()))
            n_mat = np.reshape(n_mat, (1, n_mat.shape[0]))
            np.savetxt(self.state_file, n_mat, delimiter=',', fmt='%10.5f')

            cov = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, 10,
                               torch.diagonal(state_cov_torch).cpu().numpy()))
            cov = np.reshape(cov, (1, cov.shape[0]))
            np.savetxt(self.state_file, cov, delimiter=',', fmt='%10.5f')

            state = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, -1,
                               self.LIOEKF._getNavState().pos, self.LIOEKF._getNavState().vel, self.LIOEKF._getNavState().euler,
                              self.LIOEKF._getNavState().imuerror.gyrbias, self.LIOEKF._getNavState().imuerror.accbias))
            state = np.reshape(state, (1, state.shape[0]))
            np.savetxt(self.state_file, state, delimiter=',', fmt='%10.5f')

            delta_x_info = np.array([self.LIOEKF._imu_t_, self.dataset.frame_id, 0])
            delta_x_info = np.reshape(delta_x_info, (1, delta_x_info.shape[0]))
            delta_x_torch_re = np.reshape(delta_x_torch.cpu().numpy(), (1, delta_x_torch.shape[0]))
            np.savetxt(self.state_file, np.hstack((delta_x_info, delta_x_torch_re)), delimiter=',', fmt='%10.5f')
            self.LIOEKF._stateFeedback()

            state = np.hstack((self.LIOEKF._imu_t_, self.dataset.frame_id, 1,
                               self.LIOEKF._getNavState().pos, self.LIOEKF._getNavState().vel, self.LIOEKF._getNavState().euler,
                              self.LIOEKF._getNavState().imuerror.gyrbias, self.LIOEKF._getNavState().imuerror.accbias))
            state = np.reshape(state, (1, state.shape[0]))
            np.savetxt(self.state_file, state, delimiter=',', fmt='%10.5f')
            ##########################################################################################

            if ( torch.norm(delta_x_torch - last_dx) < 0.001):
                continue
            last_dx = delta_x_torch
            self.LIOEKF._delta_x_ = np.zeros(15)

        state_cov_torch -= KH @ state_cov_torch
        self.LIOEKF._Cov_ = state_cov_torch.cpu().numpy()

        self.LIOEKF._last_update_t_ = self.LIOEKF._lidar_t_
        return delta_x_torch, None





















####################################### util or old funcs ##################################################################

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

    # def update_EKF_SDF(self, residual: torch.tensor, jacobian: torch.tensor):
    #     last_dx = torch.zeros(1, 15, device=self.config.device, dtype=self.config.tran_dtype)
    #     weight = 1000
    #     j = 1
    #     print("update_EKF_SDF")
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

    #         self.LIOEKF._delta_x_ = delta_x_torch.cpu().numpy()
    #         # print(self.LIOEKF._delta_x_)
    #         self.LIOEKF._stateFeedback()

    #         # if ((delta_x_ - last_dx).norm() < 0.001) {
    #         # break;
    #         # }
    #         # last_dx = delta_x_;
    #         self.LIOEKF._delta_x_ = np.zeros(15)


    #     state_cov_torch -= KH @ state_cov_torch
    #     self.LIOEKF._Cov_ = state_cov_torch.cpu().numpy()

    #     self.LIOEKF._last_update_t_ = self.LIOEKF._lidar_t_
    #     return delta_x_torch