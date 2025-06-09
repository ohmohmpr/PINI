import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

import copy

import LIOEKF_pybind
from utils.lio_para import LIO_Parameters
from utils.tools import transfrom_to_homo
import open3d as o3d
import os 

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
    def __init__(self, config, LIOPara, o3d_vis,
                 dataset): #debug

        self.LIOEKF = LIOEKF_pybind._LIOEKF(LIOPara)
        self.LIOEKF._openResults()
        self.LIOEKF._init()
        self.LIOPara = LIOPara
        self.config = config
        self.config.sensor_fusion = self.config.sensor_fusion

        self.dataset = dataset # debug

        # Visual
        self.o3d_vis = o3d_vis
        self.imu = o3d.geometry.TriangleMesh()

        #
        self.lidar_updated_ = self.LIOEKF.lidar_updated_
        self.pre_pose = np.identity(4)

        #
        self.pose_lidar_torch = None
        self.T_imu_NED = np.linalg.inv(transfrom_to_homo(LIOPara.imu_tran_R))
        self.T_imu_LiDAR = LIOPara.ext_imu_main
        self.T_LiDAR_NED = np.linalg.inv(self.T_imu_LiDAR) @ self.T_imu_NED # please check

        self.config = config
        path_debug = os.path.join(self.config.run_path, "EKF_ohm.txt")
        self.debug_file = open(path_debug, "w+")
        np.printoptions(precision=10)


    # util
    def logger(self, function_name, line):
        self.debug_file.write(function_name + ", " + line + '\n')

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
        self.LIOEKF.lidar_updated_(boolean)

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

        rot_euler_NED = R.from_matrix(pose_NED[:3, :3])
        euler_NED = rot_euler_NED.as_euler("zyx",degrees=True)
        return pose_NED, vel_NED, euler_NED

    def get_bodystate_LiDAR(self, Bodystate):
        pose_NED, vel_NED, _ = self.get_bodystate(Bodystate)

        pose_LiDAR = self.T_LiDAR_NED @ pose_NED
        rot_euler_LiDAR = R.from_matrix(pose_LiDAR[:3, :3])
        euler_LiDAR = rot_euler_LiDAR.as_euler("zyx",degrees=True)

        vel_LiDAR = self.T_LiDAR_NED[:3, :3] @ vel_NED
        return pose_LiDAR, vel_LiDAR, euler_LiDAR
    

    def get_bodystate_for_prediction(self, Bodystate):
        pose_LiDAR, vel_LiDAR, _ = self.get_bodystate_LiDAR(Bodystate)

        
        T_W_lidar = np.reshape(self.config.sensor_fusion['main_sensor']['lidar']['extrinsic_main_main'], (4, 4))
        pred_pose_LiDAR = pose_LiDAR @ transfrom_to_homo(self.LIOPara.imu_tran_R)  @ self.T_imu_LiDAR 
        # pred_pose_LiDAR = pose_LiDAR @ transfrom_to_homo(self.LIOPara.imu_tran_R)  @ self.T_imu_LiDAR @ T_W_lidar
        pred_vel_LiDAR = vel_LiDAR @ transfrom_to_homo(self.LIOPara.imu_tran_R)[:3, :3] @ self.T_imu_LiDAR[:3, :3]


        return pred_pose_LiDAR, pred_vel_LiDAR, None

    # main
    def newImuProcess_EKF(self):
        self.LIOEKF._newImuProcess()


    def newImuProcess_ohm(self):

        if (self.LIOEKF._is_first_imu_):
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._imupre_ = self.LIOEKF._imucur_
            self.LIOEKF._imu_t_ = self.LIOEKF._imucur_.timestamp
            self.LIOEKF._is_first_imu_ = False

            # Vis
            pose_LiDAR, vel_LiDAR, euler_LiDAR = self.get_bodystate_for_prediction(self.LIOEKF._bodystate_cur_)

            ######### logging ##########
            self.debug_file.write("frame id: " + str(self.dataset.processed_frame) + ", ")
            self.debug_file.write("topic: " + self.LIOPara.topic + ", \n")
            self.debug_file.write("ext_imu_main: " + "\n")
            np.savetxt(fname=self.debug_file, X=self.LIOPara.ext_imu_main, fmt='%1.10f')
            self.debug_file.write("\n")
            ######### logging ##########

            ######### logging ##########
            self.debug_file.write("frame id: " + str(self.dataset.processed_frame) + ", ")
            self.debug_file.write("ts: " + str(self.LIOEKF._imucur_.timestamp) + ", \n")
            self.debug_file.write("      init_pose: " + ", \n")
            np.savetxt(fname=self.debug_file, X=pose_LiDAR, fmt='%1.10f')
            self.debug_file.write("      pose: " + ", \n")
            np.savetxt(fname=self.debug_file, X=pose_LiDAR, fmt='%1.10f')
            self.debug_file.write("      vec: " + str(vel_LiDAR)+ "\n\n")
            ######### logging ##########

            self.o3d_vis._update_geometries_EKF(pose_LiDAR)
            self.o3d_vis.stop()
            return

        # set update time as the lidar time stamp
        # double
        updatetime = self.LIOEKF._lidar_t_

        lidarUpdateFlag = 0

        # Vis
        pose_LiDAR, vel_LiDAR, euler_LiDAR = self.get_bodystate_for_prediction(self.LIOEKF._bodystate_cur_)

        ######### logging ##########
        self.debug_file.write("frame id: " + str(self.dataset.processed_frame) + ", ")
        self.debug_file.write("ts: " + str(self.LIOEKF._imucur_.timestamp) + ", \n")
        self.debug_file.write("      init_pose: " + ", \n")
        np.savetxt(fname=self.debug_file, X=pose_LiDAR, fmt='%1.10f')
        self.debug_file.write("      pose: " + ", \n")
        np.savetxt(fname=self.debug_file, X=pose_LiDAR, fmt='%1.10f')
        self.debug_file.write("      vec: " + str(vel_LiDAR)+ "\n\n")
        ######### logging ##########

        self.o3d_vis._update_geometries_EKF(pose_LiDAR)

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
            else:
                self.LIOEKF._lidarUpdate()

            self.LIOEKF.lidar_updated_ = True

            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        elif (lidarUpdateFlag == 2):
            # lidardata is near current imudata, we should firstly propagate navigation state
            self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)

            if self.LIOEKF._is_first_lidar_:
                self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
            else:
                self.LIOEKF._lidarUpdate()

            self.LIOEKF.lidar_updated_ = True
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
                self.LIOEKF._lidarUpdate()
            self.LIOEKF.lidar_updated_ = True

            # propagate navigation state for the second half imudata
            self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
            self.LIOEKF._statePropagation(midimu, self.LIOEKF._imucur_)

        # check diagonal elements of current covariance matrix
        self.LIOEKF._checkStateCov()

        # update system state and imudata at the previous epoch
        self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        self.LIOEKF._imupre_ = self.LIOEKF._imucur_

    # def newImuProcess(self, dataset, tracker, config, topic):

        # cur_pose_torch = None
        # cur_odom_cov = None 
        # weight_pc_o3d = None
        # valid_flag = None
        # if (self.LIOEKF._is_first_imu_):

        #     last_pose_T_NED = self.T_NED_LiDAR @ dataset.last_pose_ref
        #     last_vel_T_NED = self.T_NED_LiDAR[:3, :3] @ dataset.last_velocity
        #     self.LIOEKF._setBodyStateCurrent(last_pose_T_NED, last_vel_T_NED)

        #     ################################ Logger ################################
        #     self.logger("newImuProcess() _is_first_imu_", 
        #                 "processed_frame: " + str(dataset.processed_frame) + 
        #                 ", imupre ts: " + str(self.LIOEKF._imupre_.timestamp) + 
        #                 ", LiDAR ts: " + str(self.LIOEKF._lidar_t_) + 
        #                 ", imucur ts: " + str(self.LIOEKF._imucur_.timestamp)
        #                 )
        #     pos_pre_LiDAR, vel_pre_LiDAR, euler_pre_LiDAR = self.get_bodystate_LiDAR(self.LIOEKF._bodystate_pre_)
        #     self.logger("newImuProcess() _is_first_imu_", 
        #                 "processed_frame: " + str(dataset.processed_frame) + 
        #                 ", pos_pre_LiDAR: " + str(pos_pre_LiDAR[:3, 3]) + 
        #                 ", vel_pre_LiDAR: " + str(vel_pre_LiDAR) + 
        #                 ", euler_pre_LiDAR: " + str(euler_pre_LiDAR)
        #                 )
        #     pos_cur_LiDAR, vel_cur_LiDAR, euler_cur_LiDAR = self.get_bodystate_LiDAR(self.LIOEKF._bodystate_cur_)
        #     self.logger("newImuProcess() _is_first_imu_", 
        #                 "processed_frame: " + str(dataset.processed_frame) + 
        #                 ", pos_cur_LiDAR: " + str(pos_cur_LiDAR[:3, 3]) + 
        #                 ", vel_cur_LiDAR: " + str(vel_cur_LiDAR) + 
        #                 ", euler_cur_LiDAR: " + str(euler_cur_LiDAR)
        #                 )
        #     ################################ Logger ################################

        #     self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        #     self.LIOEKF._imupre_ = self.LIOEKF._imucur_
        #     self.LIOEKF._imu_t_ = self.LIOEKF._imucur_.timestamp
        #     self.LIOEKF._is_first_imu_ = False
        #     return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag

        # # set update time as the lidar time stamp
        # # double
        # updatetime = self.LIOEKF._lidar_t_

        # lidarUpdateFlag = 0

        # if (self.LIOEKF._lidar_t_ > self.LIOEKF._last_update_t_ + 0.001 or self.LIOEKF._is_first_lidar_):
        #     lidarUpdateFlag = self.LIOEKF._isToUpdate(self.LIOEKF._imupre_.timestamp, self.LIOEKF._imucur_.timestamp, updatetime)

        # ################################ Logger ################################
        # self.logger("newImuProcess()", 
        #             "processed_frame: " + str(dataset.processed_frame) + 
        #             ", imupre ts: " + str(self.LIOEKF._imupre_.timestamp) + 
        #             ", LiDAR ts: " + str(updatetime) + 
        #             ", imucur ts: " + str(self.LIOEKF._imucur_.timestamp) +
        #             ", lidarUpdateFlag: " + str(lidarUpdateFlag))
        # ################################ Logger ################################
        # if (lidarUpdateFlag == 0):
        #     # only propagate navigation state
        #     self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        #     ################################ Logger ################################
        #     pos_pre_LiDAR, vel_pre_LiDAR, euler_pre_LiDAR = self.get_bodystate_LiDAR(self.LIOEKF._bodystate_pre_)
        #     self.logger("lidarUpdateFlag == 0", 
        #                 "processed_frame: " + str(dataset.processed_frame) + 
        #                 ", pos_pre_LiDAR: " + str(pos_pre_LiDAR[:3, 3]) + 
        #                 ", vel_pre_LiDAR: " + str(vel_pre_LiDAR) + 
        #                 ", euler_pre_LiDAR: " + str(euler_pre_LiDAR)
        #                 )
        #     pos_cur_LiDAR, vel_cur_LiDAR, euler_cur_LiDAR = self.get_bodystate_LiDAR(self.LIOEKF._bodystate_cur_)
        #     self.logger("lidarUpdateFlag == 0", 
        #                 "processed_frame: " + str(dataset.processed_frame) + 
        #                 ", pos_cur_LiDAR: " + str(pos_cur_LiDAR[:3, 3]) + 
        #                 ", vel_cur_LiDAR: " + str(vel_cur_LiDAR) + 
        #                 ", euler_cur_LiDAR: " + str(euler_cur_LiDAR)
        #                 )
        #     self.logger("_statePropagation()", 
        #                 "processed_frame: " + str(dataset.processed_frame) + 
        #                 ", _delta_x_: " + str(self.LIOEKF._delta_x_))
        #     ################################ Logger ################################
        # elif (lidarUpdateFlag == 1):
        #     #  lidardata is near to the previous imudata, we should firstly do lidar
        #     #  update

        #     if (self.LIOEKF._is_first_lidar_): 
        #         self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
        #         ###########################################################################################
        #         cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
        #         ###########################################################################################
        #         # self.LIOEKF._lidarUpdate()
        #     else:
        #         ###########################################################################################
        #         cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
        #         ###########################################################################################
        #         # self.LIOEKF._lidarUpdate()

        #     self.LIOEKF.lidar_updated_ = True

        #     self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        #     self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)
        # elif (lidarUpdateFlag == 2):
        #     # lidardata is near current imudata, we should firstly propagate navigation state
        #     self.LIOEKF._statePropagation(self.LIOEKF._imupre_, self.LIOEKF._imucur_)

        #     if self.LIOEKF._is_first_lidar_:
        #         self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
        #         ###########################################################################################
        #         cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
        #         ###########################################################################################
        #         # self.LIOEKF._lidarUpdate()
        #     else:
        #         ###########################################################################################
        #         cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
        #         ##########################################################################################
        #         # self.LIOEKF._lidarUpdate()

        #     self.LIOEKF.lidar_updated_ = True
        # elif (lidarUpdateFlag == 3):
        #     # lidardata is between the two imudata, we interpolate current imudata to lidar time
        #     midimu = LIOEKF_pybind._IMU()
        #     LIOEKF_pybind._imuInterpolate(self.LIOEKF._imupre_, self.LIOEKF._imucur_, updatetime, midimu)

        #     # propagate navigation state for the first half imudata
        #     self.LIOEKF._statePropagation(self.LIOEKF._imupre_, midimu)

        #     # do lidar position update at the whole second and feedback system states
        #     if (self.LIOEKF._is_first_lidar_):
        #         self.LIOEKF._initFirstLiDAR(lidarUpdateFlag)
        #         ###########################################################################################
        #         cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
        #         ###########################################################################################
        #         # self.LIOEKF._lidarUpdate()
        #     else:
        #         ###########################################################################################
        #         cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag = self.update_ohm(dataset, tracker, config, topic)
        #         ###########################################################################################
        #         # self.LIOEKF._lidarUpdate()
        #     self.LIOEKF.lidar_updated_ = True

        #     # propagate navigation state for the second half imudata
        #     self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        #     self.LIOEKF._statePropagation(midimu, self.LIOEKF._imucur_)

        # # check diagonal elements of current covariance matrix
        # self.LIOEKF._checkStateCov()

        # # update system state and imudata at the previous epoch
        # self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_
        # self.LIOEKF._imupre_ = self.LIOEKF._imucur_
        # return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag


    def update_ohm(self, dataset, tracker, config, topic):
        
        kw = np.asarray(self.LIOEKF._processScanPin())
        KeyPoints_w_torch = torch.tensor(
                kw, dtype=config.dtype, device=config.device
        )

        pos_cur_LiDAR, vel_cur_LiDAR, euler_cur_LiDAR = self.get_bodystate_LiDAR(self.LIOEKF._bodystate_cur_)
        ################################ Logger ################################
        self.logger("predicted() _is_first_imu_", 
                    "processed_frame: " + str(dataset.processed_frame) + 
                    ", pos_cur_LiDAR: " + str(pos_cur_LiDAR[:3, 3]) + 
                    ", vel_cur_LiDAR: " + str(vel_cur_LiDAR) + 
                    ", euler_cur_LiDAR: " + str(euler_cur_LiDAR)
                    )
        ################################ Logger ################################
        predicted_pos_LiDAR = pos_cur_LiDAR
        predicted_pos_LiDAR_torch = torch.tensor(predicted_pos_LiDAR, device=config.device, dtype=config.tran_dtype)


        if self.pose_lidar_torch == None:
            self.pose_lidar_torch = dataset.cur_pose_guess_torch
        else:
            self.pose_lidar_torch = predicted_pos_LiDAR_torch
        tracking_result = tracker.tracking(KeyPoints_w_torch, self.pose_lidar_torch, 
                                        dataset.cur_source_colors)
        
        cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag, sdf_res, J_mat = tracking_result

        last_pose_T_NED = self.T_NED_LiDAR @ cur_pose_torch.cpu().numpy()
        self.LIOEKF._setPoseBodyStateCurrent(last_pose_T_NED)
        ################################ Logger ################################
        pos_cur_LiDAR, vel_cur_LiDAR, euler_cur_LiDAR = self.get_bodystate_LiDAR(self.LIOEKF._bodystate_cur_)
        self.logger("updated() _is_first_imu_", 
                    "processed_frame: " + str(dataset.processed_frame) + 
                    ", pos_cur_LiDAR: " + str(pos_cur_LiDAR[:3, 3]) + 
                    ", vel_cur_LiDAR: " + str(vel_cur_LiDAR) + 
                    ", euler_cur_LiDAR: " + str(euler_cur_LiDAR)
                    )
        ################################ Logger ################################
        return cur_pose_torch, cur_odom_cov, weight_pc_o3d, valid_flag


    # def update(self, residual: torch.tensor, jacobian: torch.tensor, config):
    #     # check if residual and jacobian have same rows
    #     assert residual.shape[0] == jacobian.shape[0]

    #     POS_ID = 0
    #     VEL_ID = 3
    #     ATT_ID = 6
    #     H = torch.zeros(jacobian.shape[0], 15, device=config.device, dtype=config.tran_dtype)
    #     H[:, POS_ID:POS_ID+3] = jacobian[:, 3:6]
    #     H[:, ATT_ID:ATT_ID+3] = jacobian[:, 0:3]

    #     R = torch.eye(residual.shape[0], device=config.device, dtype=config.tran_dtype) * 1000

    #     state_cov_torch = torch.tensor(self.LIOEKF._Cov_, device=config.device, dtype=config.tran_dtype)

    #     HTRH = H.T @ R @ H
    #     HTRz = H.T @ R @ residual.to(dtype=torch.float64)

    #     S_inv = torch.inverse(HTRH + torch.inverse(state_cov_torch))
    #     delta_x_torch = S_inv @ HTRz
    #     state_cov_torch = state_cov_torch - S_inv @ HTRH @ state_cov_torch
    #     self.LIOEKF._delta_x_ = delta_x_torch.cpu().numpy()
    #     self.LIOEKF._Cov_ = state_cov_torch.cpu().numpy()

    #     print(f"delta_x: {self.LIOEKF._delta_x_}")

    #     # self.stateFeedback()
    #     self.LIOEKF._stateFeedback()
    #     # self.delta_x = np.zeros((15, 1))
    #     self.LIOEKF._delta_x_ = np.zeros((15, 1))

    #     # self.prebodystate = copy.copy(self.curbodystate)
    #     self.LIOEKF._bodystate_pre_ = self.LIOEKF._bodystate_cur_

    #     state = self.LIOEKF._getNavState_pin()
    #     pos_T_NED = np.zeros((4, 4))
    #     pos_T_NED[:3, :3] = state.rot
    #     pos_T_NED[:3, 3] = state.pos
    #     pos_T_NED[3, 3] = 1

    #     # print("pos_T_NED\n", pos_T_NED)
    #     pos_T_LiDAR = self.T_LiDAR_NED @ pos_T_NED
    #     # return lidar pose and cov
    #     return torch.tensor(pos_T_LiDAR, device=config.device, dtype=config.tran_dtype)
