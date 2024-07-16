import numpy as np
import gtsam
from scipy.spatial.transform import Rotation

from utils.config import Config

class IMUManager:
    def __init__(self, config: Config):
        
        self.config = config

        self.silence = config.silence

        self.gravity = 9.81 # read from config
        self.params = gtsam.PreintegrationCombinedParams.MakeSharedU(self.gravity)

        self.velocity = np.array([0, 0, 0])

        self.stable = False

        # for ouster imu
        self.acc_cov = 0.001249
        self.gyr_cov = 0.000208
        self.ba_cov = 0.000106
        self.bg_cov = 0.000004
    
    def init_preintegration(self, init_imuinter, gravity_align = False):
        self.T_Wi_I0, self.accBias, self.gyroBias, self.accel_sigma, self.gyro_sigma = self.imu_calibration_online(init_imuinter, gravity_align=gravity_align)

        self.imu_bias = gtsam.imuBias.ConstantBias(self.accBias, self.gyroBias)

        eye3 = np.eye(3)
        self.params.setGyroscopeCovariance(self.gyro_sigma**2 * eye3)
        self.params.setAccelerometerCovariance(self.accel_sigma**2 * eye3)

        # self.params.setGyroscopeCovariance(self.gyr_cov * eye3)
        # self.params.setAccelerometerCovariance(self.acc_cov * eye3)

        self.params.setIntegrationCovariance(1e-6 * eye3)  # 1e-3**2 * eye3 # 1e-5 * eye3
        # self.params.setBiasOmegaCovariance(self.ba_cov * eye3)
        # self.params.setBiasAccCovariance(self.ba_cov * eye3)

        self.params.setOmegaCoriolis(np.zeros(3, dtype=float))

        self.pim = gtsam.PreintegratedCombinedMeasurements(self.params, self.imu_bias)

    
    def preintegration(self, acc, gyro, dts, last_pose):

        initial_pose = gtsam.Pose3(last_pose) # under imu frame
        initial_state = gtsam.NavState(initial_pose, self.velocity)

        self.cur_frame_imu_prediction_poses = np.empty((len(dts), 4, 4)) # the prediction of imu pose wrt Wi between 2 lidar frames # pose at each interval
        # for i,dt in enumerate(dts):
        iter_count = 0 
        for cur_acc, cur_gyro, cur_dt in zip(acc, gyro, dts):
            self.pim.integrateMeasurement(cur_acc, cur_gyro, cur_dt)

            cur_imu_prediction = self.pim.predict(initial_state, self.imu_bias) # get current integration result

            cur_preintegration_pose = cur_imu_prediction.pose()
            
            cur_tran = np.eye(4)
            cur_tran[:3,:3] = cur_preintegration_pose.rotation().matrix()
            cur_tran[:3,3] = cur_preintegration_pose.translation()
            self.cur_frame_imu_prediction_poses[iter_count] = cur_tran # pose at this imu ts
            
            cur_velocity = cur_imu_prediction.velocity()
            iter_count += 1

        self.velocity = cur_velocity 
        integrated_pose = self.cur_frame_imu_prediction_poses[-1] # this is under imu frame

        # print(self.cur_frame_imu_prediction_poses) # all imu integration results under imu world frame

        # print(self.imu_bias) # the bias is changing too fast, something must be wrong # TODO

        return integrated_pose
    
    # assume the system is static in the begining
    def imu_calibration_online(self, imu_curinter, gravity_align=True):

        num_samples = len(imu_curinter['acc'])

        # Calculate averages
        if num_samples > 0:
            accel_avg = np.mean(imu_curinter['acc'], axis=0)
            gyro_avg = np.mean(imu_curinter['gyro'], axis=0)

        grav_vec = np.array([0, 0, self.gravity])

        if gravity_align:
            # Calculate initial orientation from gravity
            grav_dir = accel_avg / np.linalg.norm(accel_avg) # (normalize to avoid scale, only rot needed)
            grav_target = np.array([0, 0, 1])  # Z-up coordinate system

            # Calculate angles - Z-axis is up (z = 1 in gravity vector)
            pitch = np.arcsin(-grav_dir[0])  # rotation around y-axis
            roll = np.arcsin(grav_dir[1] / np.cos(pitch))  # rotation around x-axis
            # Create rotation matrices for roll and pitch
            roll_matrix = Rotation.from_euler('x', roll).as_matrix()
            pitch_matrix = Rotation.from_euler('y', pitch).as_matrix()
            imu_init_rotation_matrix = pitch_matrix @ roll_matrix

            # Apply rotation matrix to the calibrated initial pose
            calibrated_initial_pose = np.eye(4)
            calibrated_initial_pose[:3, :3] = imu_init_rotation_matrix
            
            if not self.silence:
                print("IMU initial pose with gravity alignment")
                print(calibrated_initial_pose)

            # Compute biases adjusted by initial pose
            grav_corrected = np.linalg.inv(imu_init_rotation_matrix) @ np.array([0, 0, self.gravity])
            accel_bias = accel_avg - grav_corrected
            gyro_bias = gyro_avg

        else:
            # Bias computation using average
            gyro_bias = gyro_avg
            accel_bias = accel_avg - grav_vec
            calibrated_initial_pose = np.eye(4)

        # Calculate max-min range for sigma initialization
        gyro_sigma = (imu_curinter['gyro'].max(axis=0) - imu_curinter['gyro'].min(axis=0)) / 2
        accel_sigma = (imu_curinter['acc'].max(axis=0) - imu_curinter['acc'].min(axis=0)) / 2
        
        return calibrated_initial_pose, gyro_bias, accel_bias, accel_sigma, gyro_sigma # array(3)