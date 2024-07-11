import numpy as np
import gtsam
import os
import torch

from utils.config import Config

class IMUManager:
    def __init__(self, config: Config):
        
        self.config = config

        self.silence = config.silence

        self.gravity = 9.81 # read from config
        params = gtsam.PreintegrationCombinedParams.MakeSharedU(self.gravity) # Dwrong! OXTS coordinates are defined as x = forward, y = right, z = down// see imu dataformat: forward,left,top
        
        # # load other param 
        # self.T_Wi_I0, self.accBias, self.gyroBias, self.accel_sigma, self.gyro_sigma = self.imu_calibration_ros(dataset)

        self.accBias = np.array([ 0,  2.09481966e-04, -1.50556073e-05])
        self.gyroBias = np.array([-1.35759117e-04,  1.82918979e-06, -1.39241744e-03]) #-1.39241744e-03
        self.imu_bias = gtsam.imuBias.ConstantBias(self.accBias, self.gyroBias)
        
        # Some arbitrary noise sigmas
        self.gyro_sigma = np.ones(3)*1e-3
        self.accel_sigma = np.ones(3)*1e-3

        I_3x3 = np.eye(3)
        params.setGyroscopeCovariance(self.gyro_sigma**2 * I_3x3)
        params.setAccelerometerCovariance(self.accel_sigma**2 * I_3x3)
        params.setIntegrationCovariance(1e-6 * I_3x3)  # 1e-3**2 * I_3x3 # 1e-5 * I_3x3
        # params.setBiasOmegaCovariance(1e-1**2 * I_3x3)
        # params.setBiasAccCovariance(1e-1**2 * I_3x3)

        self.pim = gtsam.PreintegratedCombinedMeasurements(params, self.imu_bias)

        self.velocity = np.array([0, 0, 0])

        self.velocity_record = []
        self.imu_v_initial = []
        self.imu_v_output = []
        self.imu_v_optimized = []

    
    def preintegration(self, acc, gyro, dts, last_pose):
        # preintegration
        # if cur_id == 1:
        #     initial_pose = gtsam.Pose3(self.T_Wi_I0) #self.imu_calib_initial_pose  # last_pose @ self.imu_calib_initial_pose  
        # else:
        #     initial_pose = gtsam.Pose3(last_pose)

        initial_pose = gtsam.Pose3(last_pose)
        initial_state = gtsam.NavState(
            initial_pose,
            self.velocity) # https://github.com/borglab/gtsam/blob/4abef9248edc4c49943d8fd8a84c028deb486f4c/python/gtsam/examples/CombinedImuFactorExample.py#L164C9-L168C46 
        
        # self.imu_v_initial.append(self.velocity) # testing

        #
        self.imu_prediction_poses_curinterval = torch.empty(len(dts), 4, 4) # the prediction of imu pose wrt Wi between 2 lidar frames # pose at each interval
        for i,dt in enumerate(dts):
            # self.pim.integrate() # https://github.com/borglab/gtsam/blob/0fee5cb76e7a04b590ff0dc1051950da5b265340/python/gtsam/examples/PreintegrationExample.py#L159C16-L159C70 
            self.pim.integrateMeasurement(acc[i], gyro[i], dt) # https://github.com/borglab/gtsam/blob/4abef9248edc4c49943d8fd8a84c028deb486f4c/python/gtsam/examples/CombinedImuFactorExample.py#L175C12-L177C74 
            
            pose_each_frame_homo = np.eye(4)
            pose_each_frame = self.pim.predict(initial_state, self.imu_bias).pose()
            pose_each_frame_homo[:3,:3] = pose_each_frame.rotation().matrix()
            pose_each_frame_homo[:3,3] = pose_each_frame.translation()
            self.imu_prediction_poses_curinterval[i] = torch.tensor(pose_each_frame_homo) # pose at this imu ts

        imu_prediction = self.pim.predict(initial_state, self.imu_bias) # final pose
        predicted_pose = imu_prediction.pose() # w2imu
        self.velocity = imu_prediction.velocity()
        self.imu_v_output.append(self.velocity)

        # not add it now
        # self.graph_initials.insert(gtsam.symbol('v', cur_id), self.velocity)
        # self.graph_initials.insert(gtsam.symbol('b', cur_id), self.imu_bias) # TODO? What bias??? # https://github.com/borglab/gtsam/blob/4abef9248edc4c49943d8fd8a84c028deb486f4c/python/gtsam/examples/CombinedImuFactorExample.py#L219C17-L221C55 
        
        predicted_pose = np.eye(4)
        predicted_pose[:3,:3] = predicted_pose.rotation().matrix()
        predicted_pose[:3,3] = predicted_pose.translation()

        # if not self.config.imu_pgo:
        #     self.pim.resetIntegration() # -- preintegration testing --- 

        return predicted_pose
        
    # def imu_calibration_ros(self, dataset: SLAMDataset, visual=False, gravity_align=True):
    #     imu_curinter = dataset.imu_curinter
    #     num_samples = len(imu_curinter['acc'])

    #     # Calculate averages
    #     if num_samples > 0:
    #         accel_avg = np.mean(imu_curinter['acc'], axis=0)
    #         gyro_avg = np.mean(imu_curinter['gyro'], axis=0)

    #     grav_vec = np.array([0, 0, self.GRAVITY])

    #     if gravity_align:
    #         # Calculate initial orientation from gravity
    #         grav_dir = accel_avg / np.linalg.norm(accel_avg) # (normalize to avoid scale, only rot needed)
    #         grav_target = np.array([0, 0, 1])  # Z-up coordinate system

    #         # Calculate angles - Z-axis is up (z = 1 in gravity vector)
    #         pitch = np.arcsin(-grav_dir[0])  # rotation around y-axis
    #         roll = np.arcsin(grav_dir[1] / np.cos(pitch))  # rotation around x-axis
    #         # Create rotation matrices for roll and pitch
    #         roll_matrix = R.from_euler('x', roll).as_matrix()
    #         pitch_matrix = R.from_euler('y', pitch).as_matrix()
            
    #         # Combine roll and pitch into a single rotation matrix
    #         rotation_matrix = pitch_matrix @ roll_matrix

    #         # Apply rotation matrix to the calibrated initial pose
    #         calibrated_initial_pose = np.eye(4)
    #         calibrated_initial_pose[:3, :3] = rotation_matrix
    #         print('-----------calibration imu initial pose ---------------',calibrated_initial_pose)

    #         # Compute biases adjusted by initial pose
    #         grav_corrected =  np.linalg.inv(calibrated_initial_pose[:3, :3]) @ np.array([0, 0, self.GRAVITY])
    #         accel_bias = accel_avg - grav_corrected
    #         gyro_bias = gyro_avg

    #     else:
    #         # # Bias computation using average
    #         gyro_bias = gyro_avg
    #         accel_bias = accel_avg - grav_vec
    #         calibrated_initial_pose = np.eye(4)

    #     timestamps = imu_curinter['dt']
    #     # Calculate max-min range for sigma initialization
    #     gyro_sigma = (imu_curinter['gyro'].max(axis=0) - imu_curinter['gyro'].min(axis=0)) / 2
    #     accel_sigma = (imu_curinter['acc'].max(axis=0) - imu_curinter['acc'].min(axis=0)) / 2
        
    #     # V- Plotting the IMU data
    #     if visual:            
    #         fig, axs = plt.subplots(2, 1, figsize=(10, 8))
    #         # Gyroscope data
    #         axs[0].plot(timestamps, imu_curinter['gyro'][:, 0], label='Gyro X', color='r')
    #         axs[0].plot(timestamps, imu_curinter['gyro'][:, 1], label='Gyro Y', color='g')
    #         axs[0].plot(timestamps, imu_curinter['gyro'][:, 2], label='Gyro Z', color='b')
    #         axs[0].axhline(gyro_bias[0], color='r', linestyle='--', label='Bias X')
    #         axs[0].axhline(gyro_bias[1], color='g', linestyle='--', label='Bias Y')
    #         axs[0].axhline(gyro_bias[2], color='b', linestyle='--', label='Bias Z')
    #         axs[0].set_title('Gyroscope Data')
    #         axs[0].set_xlabel('Time [s]')
    #         axs[0].set_ylabel('Angular Velocity [rad/s]')
    #         axs[0].legend()
    #         axs[0].grid(True)
    #         # Display gyro sigmas on the plot
    #         axs[0].text(0.05, 0.95, f'Gyro Sigma X: {gyro_sigma[0]:.6f}\nGyro Sigma Y: {gyro_sigma[1]:.6f}\nGyro Sigma Z: {gyro_sigma[2]:.6f}',
    #                 transform=axs[0].transAxes, fontsize=10, verticalalignment='top')
    #         # Accelerometer data
    #         axs[1].plot(timestamps, imu_curinter['acc'][:, 0], label='Accel X', color='r')
    #         axs[1].plot(timestamps, imu_curinter['acc'][:, 1], label='Accel Y', color='g')
    #         axs[1].plot(timestamps, imu_curinter['acc'][:, 2], label='Accel Z', color='b')
    #         axs[1].axhline(accel_bias[0], color='r', linestyle='--', label='Bias X')
    #         axs[1].axhline(accel_bias[1], color='g', linestyle='--', label='Bias Y')
    #         axs[1].axhline(accel_bias[2] + self.GRAVITY, color='b', linestyle='--', label='Bias Z + Gravity')
    #         axs[1].set_title('Accelerometer Data')
    #         axs[1].set_xlabel('Time [s]')
    #         axs[1].set_ylabel('Linear Acceleration [m/s^2]')
    #         axs[1].legend()
    #         axs[1].grid(True)
    #         # Display accel sigmas on the plot
    #         axs[1].text(0.05, 0.95, f'Accel Sigma X: {accel_sigma[0]:.6f}\nAccel Sigma Y: {accel_sigma[1]:.6f}\nAccel Sigma Z: {accel_sigma[2]:.6f}',
    #                 transform=axs[1].transAxes, fontsize=10, verticalalignment='top')

    #         plt.tight_layout()
    #         plt.show()

    #     return calibrated_initial_pose, gyro_bias, accel_bias, accel_sigma, gyro_sigma # array(3)