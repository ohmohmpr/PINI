import numpy as np
import torch
from scipy.spatial.transform import Rotation as R
import copy


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
class EKF:
    def __init__(self, config, imu_lidar_extrinsic):
        self.state_covariance = np.zeros((15, 15))
        self.noise_covariance = np.zeros((12, 12))

        # initialize 15 dimensional state vector
        self.delta_x = np.zeros((15, 1))
        
        self.prebodystate = BodyState()
        self.curbodystate = BodyState()

        self.imuerror = IMUError()

        self.curimu = IMU()
        self.preimu = IMU()

        self.stateid = StateID()
        self.noiseid = NoiseID()

        self.imu_lidar_extrinsic = imu_lidar_extrinsic

        # TODO
        # Initialize state covariance matrix and noise covariance matrix
        initposstd = np.array([0.01, 0.01, 0.01])
        initvelstd = np.array([ 0.05, 0.05, 0.05 ])
        initattstd = np.array([ 0.1, 0.1, 0.1 ]) * np.pi / 180
        initgyrobiasstd = np.array([ 200.0, 200.0, 200.0 ]) * np.pi / 180 / 3600
        initaccbiasstd = np.array([ 500.0, 500.0, 500.0 ]) * 1e-5
        arw = np.array([2, 2, 2]) * np.pi / 180 / 60
        vrw = np.array([40, 40, 40]) / 60
        coortime = 1 * 3600

        # set initial state covariance
        self.state_covariance[self.stateid.POS_ID:self.stateid.POS_ID+3, self.stateid.POS_ID:self.stateid.POS_ID+3] = create_covariance_block(initposstd)
        self.state_covariance[self.stateid.VEL_ID:self.stateid.VEL_ID+3, self.stateid.VEL_ID:self.stateid.VEL_ID+3] = create_covariance_block(initvelstd)
        self.state_covariance[self.stateid.ATT_ID:self.stateid.ATT_ID+3, self.stateid.ATT_ID:self.stateid.ATT_ID+3] = create_covariance_block(initattstd)
        self.state_covariance[self.stateid.GYRO_BIAS_ID:self.stateid.GYRO_BIAS_ID+3, self.stateid.GYRO_BIAS_ID:self.stateid.GYRO_BIAS_ID+3] = create_covariance_block(initgyrobiasstd)
        self.state_covariance[self.stateid.ACC_BIAS_ID:self.stateid.ACC_BIAS_ID+3, self.stateid.ACC_BIAS_ID:self.stateid.ACC_BIAS_ID+3] = create_covariance_block(initaccbiasstd)

        # set noise covariance
        self.noise_covariance[self.noiseid.VEL_RANDOMWALK_ID:self.noiseid.VEL_RANDOMWALK_ID+3, self.noiseid.VEL_RANDOMWALK_ID:self.noiseid.VEL_RANDOMWALK_ID+3] = create_covariance_block(vrw)
        self.noise_covariance[self.noiseid.ANGEL_RANDOMWALK_ID:self.noiseid.ANGEL_RANDOMWALK_ID+3, self.noiseid.ANGEL_RANDOMWALK_ID:self.noiseid.ANGEL_RANDOMWALK_ID+3] = create_covariance_block(arw)
        self.noise_covariance[self.noiseid.GYRO_BIAS_STD_ID:self.noiseid.GYRO_BIAS_STD_ID+3, self.noiseid.GYRO_BIAS_STD_ID:self.noiseid.GYRO_BIAS_STD_ID+3] = create_covariance_block(initgyrobiasstd) * 2 / coortime
        self.noise_covariance[self.noiseid.ACC_BIAS_STD_ID:self.noiseid.ACC_BIAS_STD_ID+3, self.noiseid.ACC_BIAS_STD_ID:self.noiseid.ACC_BIAS_STD_ID+3] = create_covariance_block(initaccbiasstd) * 2 / coortime

    def convertIMU(self, imu_data):
        imu = IMU()
        imu.dt = imu_data['dt']
        imu.timestamp = imu_data['ts']
        imu.gyro = imu_data['gyro']
        imu.acc = imu_data['acc']
        return imu
    
    def initIMUbodystate(self, cur_lidar_pose_torch: torch.Tensor, frame_imu_data, frame_ts):
        cur_lidar_pose = cur_lidar_pose_torch.cpu().numpy()

        trans_lidar_imu = np.copy(self.imu_lidar_extrinsic)
        trans_lidar_imu[:3,:3] = trans_lidar_imu[:3,:3].T
        trans_lidar_imu[:3,3] = - trans_lidar_imu[:3,:3] @ trans_lidar_imu[:3,3]

        cur_lidar_pose_inv = np.linalg.inv(cur_lidar_pose)

        self.curbodystate.pose = np.eye(4)
        self.curbodystate.vel = np.zeros(3)

        #Transformation from lidar world frame to imu world frame T_wl^wi
        self.trans_wi_wl = self.curbodystate.pose @ self.imu_lidar_extrinsic @ cur_lidar_pose_inv

        self.prebodystate = copy.copy(self.curbodystate)

        imu_ts = frame_imu_data['ts']

        # 计算每个时间戳与目标时间戳之间的差值
        time_diffs = np.abs(imu_ts - frame_ts)  

        # 找到差值最小的那个索引
        closest_index = np.argmin(time_diffs)

        # 使用该索引从IMU数据中提取相应的数据条目
        closest_imu_data = {key: value[closest_index] for key, value in frame_imu_data.items()}

        self.curimu = self.convertIMU(closest_imu_data)
        self.preimu = copy.copy(self.curimu)

    
    def processIMU(self, frame_imu_data):
        # iterate frame_imu_data
        for i in range(len(frame_imu_data['ts'])):
            
            tmp_imu = self.convertIMU({key: value[i] for key, value in frame_imu_data.items()})
            if tmp_imu.timestamp <= self.curimu.timestamp:
                self.preimu = self.curimu
                continue
            self.curimu = tmp_imu
            self.imuCompensate()
            # print(f"preimu ts: {self.preimu.timestamp} gyro: {self.preimu.gyro}, acc: {self.preimu.acc}, dt: {self.preimu.dt}")
            # print(f"curimu ts: {self.curimu.timestamp} gyro: {self.curimu.gyro}, acc: {self.curimu.acc}, dt: {self.curimu.dt}")

            self.statePropagation()
            self.prebodystate = copy.copy(self.curbodystate)
            self.preimu = self.curimu
             
            lidarpose = self.getcurLidarpose()
            lidar_att = R.from_matrix(lidarpose[:3,:3]).as_euler('xyz')*180/np.pi
            lidar_pos = lidarpose[:3,3]
            imu_att = R.from_matrix(self.curbodystate.pose[:3,:3]).as_euler('xyz')*180/np.pi
            imu_pos = self.curbodystate.pose[:3,3]
            # print(f"ts: {self.curimu.timestamp} lidar pos: {lidar_pos}, lidar att: {lidar_att}")
            # print(f"ts: {self.curimu.timestamp} imu: {self.curimu.gyro} {self.curimu.acc} dt: {self.curimu.dt}")
            print(f"ts: {self.curimu.timestamp} imu   pos: {imu_pos}, imu   att: {imu_att} imu vel: {self.curbodystate.vel}")

            # print the diagonal of state covariance
            print(f"state covariance: {np.diag(self.state_covariance)}")

    def getcurLidarpose(self) -> np.array:
        lidarpose = np.linalg.inv(self.trans_wi_wl) @ self.curbodystate.pose @ self.imu_lidar_extrinsic
        return lidarpose
    
    
    def imuCompensate(self):
        self.curimu.gyro = self.curimu.gyro - self.imuerror.gyro_bias
        self.curimu.acc = self.curimu.acc - self.imuerror.acc_bias

    
    def insMechanization(self):
        normG = 9.8
        curimu_dvel = self.curimu.acc * self.curimu.dt
        curimu_dtheta = self.curimu.gyro * self.curimu.dt
        preimu_dvel = self.preimu.acc * self.preimu.dt
        preimu_dtheta = self.preimu.gyro * self.preimu.dt

        temp1 = np.cross(curimu_dtheta, curimu_dvel) / 2
        temp2 = np.cross(preimu_dtheta, curimu_dvel) / 12
        temp3 = np.cross(preimu_dvel, curimu_dtheta) / 12

        d_vfb = curimu_dvel + temp1 + temp2 + temp3

        d_vfn = self.prebodystate.pose[:3,:3] @ d_vfb

        gl = np.array([0, 0, normG])
        d_vgn = gl * self.curimu.dt

        self.curbodystate.vel = self.prebodystate.vel + d_vfn + d_vgn

        midvel = (self.curbodystate.vel + self.prebodystate.vel) / 2
        self.curbodystate.pose[:3,3] = self.prebodystate.pose[:3,3] + midvel * self.curimu.dt

        rot_bframe = curimu_dtheta + np.cross(preimu_dtheta, curimu_dtheta) / 12

        self.curbodystate.pose[:3, :3] = self.prebodystate.pose[:3,:3] @ R.from_rotvec(rot_bframe).as_matrix()
    


    def statePropagation(self):
        self.insMechanization()
        
        state_transition_mat = np.eye(15)
        jacobian = np.zeros((15, 15))
        process_noise_cov = np.zeros((15, 15))
        noise_driven_mat = np.zeros((15, 12))

        pre_rotation_mat = self.prebodystate.pose[:3,:3]

        jacobian[self.stateid.POS_ID:self.stateid.POS_ID+3, self.stateid.VEL_ID:self.stateid.VEL_ID+3] = np.eye(3)
        jacobian[self.stateid.VEL_ID:self.stateid.VEL_ID+3, self.stateid.ATT_ID:self.stateid.ATT_ID+3] = skew_symmetric(pre_rotation_mat @ self.curimu.acc)
        jacobian[self.stateid.VEL_ID:self.stateid.VEL_ID+3, self.stateid.ACC_BIAS_ID:self.stateid.ACC_BIAS_ID+3] = pre_rotation_mat
        jacobian[self.stateid.ATT_ID:self.stateid.ATT_ID+3, self.stateid.GYRO_BIAS_ID:self.stateid.GYRO_BIAS_ID+3] = -1.0 * pre_rotation_mat
        

        noise_driven_mat[self.stateid.VEL_ID:self.stateid.VEL_ID+3, self.noiseid.VEL_RANDOMWALK_ID:self.noiseid.VEL_RANDOMWALK_ID+3] = pre_rotation_mat
        noise_driven_mat[self.stateid.ATT_ID:self.stateid.ATT_ID+3, self.noiseid.ANGEL_RANDOMWALK_ID:self.noiseid.ANGEL_RANDOMWALK_ID+3] = pre_rotation_mat
        noise_driven_mat[self.stateid.GYRO_BIAS_ID:self.stateid.GYRO_BIAS_ID+3, self.noiseid.GYRO_BIAS_STD_ID:self.noiseid.GYRO_BIAS_STD_ID+3] = np.eye(3)
        noise_driven_mat[self.stateid.ACC_BIAS_ID:self.stateid.ACC_BIAS_ID+3, self.noiseid.ACC_BIAS_STD_ID:self.noiseid.ACC_BIAS_STD_ID+3] = np.eye(3)

        state_transition_mat = state_transition_mat + jacobian * self.curimu.dt

        process_noise_cov = noise_driven_mat @ self.noise_covariance @ noise_driven_mat.T * self.curimu.dt
        process_noise_cov = (state_transition_mat @ process_noise_cov @ state_transition_mat.T + process_noise_cov) / 2

        self.state_covariance = state_transition_mat @ self.state_covariance @ state_transition_mat.T + process_noise_cov
    
    # TODO torch parallelization for update
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



    
    def stateFeedback(self):
        delta_translation = self.delta_x[self.stateid.POS_ID:self.stateid.POS_ID + 3]
        delta_att = self.delta_x[self.stateid.ATT_ID: self.stateid.ATT_ID + 3]
        self.curbodystate.pose[:3,:3] = R.from_rotvec(delta_att).as_matrix() * self.curbodystate.pose[:3,:3]
        self.curbodystate.pose[:3,3] -= delta_translation

        # Velocity error feedback
        delta_vel = self.delta_x[self.stateid.VEL_ID:self.stateid.VEL_ID + 3]
        self.curbodystate.vel -= delta_vel

        # IMU bias error feedback
        delta_bias_gyro = self.delta_x[self.stateid.GYRO_BIAS_ID:self.stateid.GYRO_BIAS_ID + 3]
        self.imuerror.gyro_bias += delta_bias_gyro
        delta_bias_acc = self.delta_x[self.stateid.ACC_BIAS_ID: self.stateid.ACC_BIAS_ID + 3]
        self.imuerror.acc_bias += delta_bias_acc
