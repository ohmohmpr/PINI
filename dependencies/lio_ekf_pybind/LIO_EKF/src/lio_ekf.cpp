// MIT License
//
// Copyright (c) 2024 Yibin Wu, Tiziano Guadagnino
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "lio_ekf.hpp"
#include "kiss_icp/core/Deskew.hpp"
#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "lio_types.hpp"
#include "rotation.hpp"
#include "unscendent_transform.hpp"
#include <Eigen/Sparse>
#include <algorithm>
#include <execution>
#include <fstream>
// #include <ros/ros.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

namespace {
Eigen::Vector3d SIGMA_POINT =
    Eigen::Vector3d(0.01, 0.05 * lio_ekf::D2R, 0.05 * lio_ekf::D2R);
Eigen::Matrix3d POINT_NOISE_COV = SIGMA_POINT.cwiseAbs2().asDiagonal();
constexpr double mid_pose_timestamp{0.5};
} // namespace

inline double square(double x) { return x * x; }

namespace lio_ekf {

auto create_covariance_block = [](const auto &v1) {
  return v1.cwiseAbs2().asDiagonal();
};

void LIOEKF::init() {

  imu_t_ = 0.0;
  lidar_t_ = 0.0;

  // resize covariance matrix, system noise matrix, and system error state
  // matrix

  Cov_.setZero();
  Imu_Prediction_Covariance_.setZero();
  State_Noise_Cov_.setZero();
  delta_x_.setZero();

  // initialize noise matrix
  auto imunoise = liopara_.imunoise;

  State_Noise_Cov_.block(ANGEL_RANDOMWALK_ID, ANGEL_RANDOMWALK_ID, 3, 3) =
      create_covariance_block(imunoise.angle_randomwalk);
  State_Noise_Cov_.block(VEL_RANDOMWALK_ID, VEL_RANDOMWALK_ID, 3, 3) =
      create_covariance_block(imunoise.velocity_randomwalk);
  State_Noise_Cov_.block(GYRO_BIAS_STD_ID, GYRO_BIAS_STD_ID, 3, 3) =
      2 / imunoise.correlation_time *
      create_covariance_block(imunoise.gyrbias_std);
  State_Noise_Cov_.block(ACC_BIAS_STD_ID, ACC_BIAS_STD_ID, 3, 3) =
      2 / imunoise.correlation_time *
      create_covariance_block(imunoise.accbias_std);

  // set initial state (position, velocity, attitude and IMU error) and
  // covariance
  navStateInitialization(liopara_.initstate, liopara_.initstate_std);

  is_first_imu_ = true;
  is_first_lidar_ = true;

  odomRes_ << "\nliopara_.Trans_lidar_imu_origin : \n"
           << liopara_.Trans_lidar_imu_origin << "\nliopara_.imu_tran_R : \n"
           << liopara_.imu_tran_R << "\nliopara_.Trans_lidar_imu : \n"
           << liopara_.Trans_lidar_imu << std::endl;

  odomRes_ << "\nliopara_.initstate_std.pos : \n"
           << liopara_.initstate_std.pos << "\nliopara_.initstate_std.vel : \n"
           << liopara_.initstate_std.vel
           << "\nliopara_.initstate_std.euler : \n"
           << liopara_.initstate_std.euler
           << "\nliopara_.initstate_std.imuerror.gyrbias : \n"
           << liopara_.initstate_std.imuerror.gyrbias
           << "\nliopara_.initstate_std.imuerror.accbias : \n"
           << liopara_.initstate_std.imuerror.accbias << std::endl;

  odomRes_ << "\nliopara_.imunoise.angle_randomwalk : \n"
           << liopara_.imunoise.angle_randomwalk
           << "\nliopara_.imunoise.velocity_randomwalk : \n"
           << liopara_.imunoise.velocity_randomwalk
           << "\nliopara_.imunoise.gyrbias_std : \n"
           << liopara_.imunoise.gyrbias_std
           << "\nliopara_.imunoise.accbias_std : \n"
           << liopara_.imunoise.accbias_std
           << "\nliopara_.imunoise.correlation_time : \n"
           << liopara_.imunoise.correlation_time << std::endl;

  odomRes_ << "\nliopara_.deskew : " << liopara_.deskew
           << "\nliopara_.preprocess : " << liopara_.preprocess
           << "\nliopara_.max_range : " << liopara_.max_range
           << "\nliopara_.min_range : " << liopara_.min_range
           << "\nliopara_.max_points_per_voxel : "
           << liopara_.max_points_per_voxel
           << "\nliopara_.voxel_size : " << liopara_.voxel_size
           << "\nliopara_.max_iteration : " << liopara_.max_iteration
           << std::endl;
}

void LIOEKF::navStateInitialization(const NavState &initstate,
                                    const NavState &initstate_std) {

  // initialize imu error
  imuerror_ = initstate.imuerror;

  // initialize covariance
  ImuError imuerror_std = initstate_std.imuerror;

  Cov_.block(POS_ID, POS_ID, 3, 3) = create_covariance_block(initstate_std.pos);
  Cov_.block(VEL_ID, VEL_ID, 3, 3) = create_covariance_block(initstate_std.vel);
  Cov_.block(ATT_ID, ATT_ID, 3, 3) =
      create_covariance_block(initstate_std.euler);
  Cov_.block(GYRO_BIAS_ID, GYRO_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrbias);
  Cov_.block(ACC_BIAS_ID, ACC_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.accbias);
}

Eigen::Matrix4d LIOEKF::initFirstLiDAR(const int lidarUpdateFlag) {
  Sophus::SE3d initLidarpose_w = Sophus::SE3d();
  const auto &lidar_to_imu_extrinsic = Sophus::SE3d(liopara_.Trans_lidar_imu);
  // odomRes_ << " initFirstLiDAR : " << lidarUpdateFlag << std::endl;
  switch (lidarUpdateFlag) {
  case 1: {
    initLidarpose_w = bodystate_pre_.pose * lidar_to_imu_extrinsic;
    break;
  }
  case 2: {
    initLidarpose_w = bodystate_cur_.pose * lidar_to_imu_extrinsic;
    break;
  }
  case 3: {
    const auto delta_pose = bodystate_pre_.pose.inverse() * bodystate_cur_.pose;
    double relative_lidar_timestamp = (lidar_t_ - imupre_.timestamp) /
                                      (imucur_.timestamp - imupre_.timestamp);
    const auto interpolated_vector_pose =
        relative_lidar_timestamp * delta_pose.log();
    const Sophus::SE3d interpolated_pose =
        bodystate_pre_.pose * Sophus::SE3d::exp(interpolated_vector_pose);
    initLidarpose_w = interpolated_pose * lidar_to_imu_extrinsic;
    break;
  }
  }

  lio_map_.Update(curpoints_, initLidarpose_w);
  is_first_lidar_ = false;
  last_update_t_ = lidar_t_;
  first_lidar_t = lidar_t_;
  return initLidarpose_w.matrix();
}

void LIOEKF::newImuProcess() {

  if (is_first_imu_) {
    bodystate_pre_ = bodystate_cur_;
    imupre_ = imucur_;
    imu_t_ = imucur_.timestamp;
    is_first_imu_ = false;
    return;
  }

  // set update time as the lidar time stamp
  double updatetime = lidar_t_;

  int lidarUpdateFlag = 0;

  if (lidar_t_ > last_update_t_ + 0.001 || is_first_lidar_)
    lidarUpdateFlag =
        isToUpdate(imupre_.timestamp, imucur_.timestamp, updatetime);
  // determine if we should do  update

  // odomRes_ << "isToUpdate :"
  //          << " " << lidarUpdateFlag << " "
  //          << ", imutime1 :" << imupre_.timestamp
  //          << ", updatetime :" << updatetime
  //          << ", imutime2 :" << imucur_.timestamp << std::endl;

  switch (lidarUpdateFlag) {
  case 0: {
    // only propagate navigation state
    statePropagation(imupre_, imucur_);
    break;
  }
  case 1: {
    // lidardata is near to the previous imudata, we should firstly do lidar
    // update
    if (is_first_lidar_) {
      initFirstLiDAR(lidarUpdateFlag);
    } else {
      lidarUpdate();
    }

    lidar_updated_ = true;

    bodystate_pre_ = bodystate_cur_;
    statePropagation(imupre_, imucur_);
    break;
  }
  case 2: {
    // lidardata is near current imudata, we should firstly propagate navigation
    // state
    statePropagation(imupre_, imucur_);
    if (is_first_lidar_) {
      initFirstLiDAR(lidarUpdateFlag);
    } else {
      lidarUpdate();
    }

    lidar_updated_ = true;

    break;
  }
  case 3: {
    // lidardata is between the two imudata, we interpolate current imudata to
    // lidar time
    IMU midimu;
    imuInterpolate(imupre_, imucur_, updatetime, midimu);

    // propagate navigation state for the first half imudata
    statePropagation(imupre_, midimu);

    // do lidar position update at the whole second and feedback system states
    if (is_first_lidar_) {
      initFirstLiDAR(lidarUpdateFlag);
    } else {
      lidarUpdate();
    }

    lidar_updated_ = true;
    // propagate navigation state for the second half imudata
    bodystate_pre_ = bodystate_cur_;
    statePropagation(midimu, imucur_);
    break;
  }
  }

  // check diagonal elements of current covariance matrix
  checkStateCov();

  // update system state and imudata at the previous epoch
  bodystate_pre_ = bodystate_cur_;
  imupre_ = imucur_;
}

void LIOEKF::statePropagation(IMU &imupre, IMU &imucur) {

  // debug_<<"statePropagation start"<<std::endl;
  // compensate imu error to 'imucur', 'imupre' has been compensated
  imuCompensate(imucur, imuerror_);

  // update imustate(mechanization)
  insMechanization(bodystate_pre_, bodystate_cur_, imupre, imucur);

  // system noise propagate, phi-angle error model for attitude error

  Eigen::Matrix15d State_Transition_Mat, Jacobian, Process_Noise_Cov;
  Eigen::Matrix15_12d Noise_Driven_Mat;

  State_Transition_Mat.setIdentity();
  Jacobian.setZero();
  Process_Noise_Cov.setZero();
  Noise_Driven_Mat.setZero();

  // position error
  Jacobian.block(POS_ID, VEL_ID, 3, 3) = Eigen::Matrix3d::Identity();

  // velocity error
  Jacobian.block(VEL_ID, ATT_ID, 3, 3) = Rotation::skewSymmetric(
      bodystate_pre_.pose.rotationMatrix() * imucur.linear_acceleration);
  Jacobian.block(VEL_ID, ACC_BIAS_ID, 3, 3) =
      bodystate_pre_.pose.rotationMatrix();

  // attitude error
  Jacobian.block(ATT_ID, GYRO_BIAS_ID, 3, 3) =
      -1.0 * bodystate_pre_.pose.rotationMatrix();

  // system noise driven matrix
  Noise_Driven_Mat.block(VEL_ID, VEL_RANDOMWALK_ID, 3, 3) =
      bodystate_pre_.pose.rotationMatrix();
  Noise_Driven_Mat.block(ATT_ID, ANGEL_RANDOMWALK_ID, 3, 3) =
      bodystate_pre_.pose.rotationMatrix();
  Noise_Driven_Mat.block(GYRO_BIAS_ID, GYRO_BIAS_STD_ID, 3, 3) =
      Eigen::Matrix3d::Identity();
  Noise_Driven_Mat.block(ACC_BIAS_ID, ACC_BIAS_STD_ID, 3, 3) =
      Eigen::Matrix3d::Identity();

  // compute the state transition matrix
  State_Transition_Mat.setIdentity();
  State_Transition_Mat = State_Transition_Mat + Jacobian * imucur.dt;

  // compute system propagation noise
  Process_Noise_Cov = Noise_Driven_Mat * State_Noise_Cov_ *
                      Noise_Driven_Mat.transpose() * imucur.dt;
  Process_Noise_Cov = (State_Transition_Mat * Process_Noise_Cov *
                           State_Transition_Mat.transpose() +
                       Process_Noise_Cov) /
                      2;
  Process_Noise_Cov += 1e-8 * Eigen::Matrix15d::Identity();

  ekfPredict(State_Transition_Mat, Process_Noise_Cov);
}

std::vector<Eigen::Vector3d>
LIOEKF::DeSkewScan(const std::vector<Eigen::Vector3d> &frame,
                   const std::vector<double> &timestamps,
                   const Sophus::SE3d &start_pose,
                   const Sophus::SE3d &finish_pose) {
  const auto delta_pose = (start_pose.inverse() * finish_pose).log();
  std::vector<Eigen::Vector3d> corrected_frame(frame.size());

  tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
    const auto motion =
        Sophus::SE3d::exp((timestamps[i] - mid_pose_timestamp) * delta_pose);
    corrected_frame[i] = motion * frame[i];
  });
  return corrected_frame;
}

std::vector<Eigen::Vector3d> LIOEKF::processScanPin() {
  Sophus::SE3d lidar_to_imu = Sophus::SE3d(liopara_.Trans_lidar_imu);
  Sophus::SE3d previous_pose_scan = bodystate_pre_.pose;
  Sophus::SE3d current_pose_scan = bodystate_cur_.pose;
  curpoints_w_ = DeSkewScan(curpoints_, timestamps_per_points_,
                            previous_pose_scan, current_pose_scan);

  const auto cropped_frame = kiss_icp::Preprocess(
      curpoints_w_, liopara_.max_range, liopara_.min_range);

  auto [source, frame_downsample] = Voxelize(cropped_frame);
  auto source_in_imu_frame = source;

  keypoints_w_ = source;

  return keypoints_w_;
}

std::tuple<Vector3dVector, Vector3dVector> LIOEKF::processScan() {
  Sophus::SE3d lidar_to_imu = Sophus::SE3d(liopara_.Trans_lidar_imu);
  Sophus::SE3d previous_pose_scan = bodystate_pre_.pose * lidar_to_imu;
  Sophus::SE3d current_pose_scan = bodystate_cur_.pose * lidar_to_imu;
  curpoints_w_ = DeSkewScan(curpoints_, timestamps_per_points_,
                            previous_pose_scan, current_pose_scan);
  const auto cropped_frame = kiss_icp::Preprocess(
      curpoints_w_, liopara_.max_range, liopara_.min_range);
  auto [source, frame_downsample] = Voxelize(cropped_frame);
  auto source_in_imu_frame = source;
  keypoints_w_ = source;
  TransformPoints(lidar_to_imu.matrix(), source_in_imu_frame);

  const Eigen::Matrix4d &lidar_to_imu_origin = liopara_.Trans_lidar_imu_origin;
  TransformPoints(lidar_to_imu_origin, keypoints_w_);
  TransformPoints(lidar_to_imu_origin, curpoints_w_);

  return std::make_tuple(source_in_imu_frame, frame_downsample);
}

void LIOEKF::lidarUpdate() {

  auto [source, frame_downsample] = processScan();
  Eigen::Matrix6d imu_pose_covariance = Eigen::Matrix6d::Identity();
  imu_pose_covariance.block<3, 3>(0, 0) =
      Imu_Prediction_Covariance_.block<3, 3>(0, 0);
  imu_pose_covariance.block<3, 3>(3, 3) =
      Imu_Prediction_Covariance_.block<3, 3>(6, 6);
  imu_pose_covariance.block<3, 3>(0, 3) =
      Imu_Prediction_Covariance_.block<3, 3>(0, 6);
  imu_pose_covariance.block<3, 3>(3, 0) =
      Imu_Prediction_Covariance_.block<3, 3>(6, 0);
  const auto relative_pose =
      bodystate_pre_.pose.inverse() * bodystate_cur_.pose;
  const auto initial_guess = bodystate_cur_.pose;
  auto square = [](const double &x) { return x * x; };
  double uncertanty_motion =
      propagateUscendent(relative_pose, imu_pose_covariance);
  double map_uncertanty =
      square(liopara_.voxel_size / std::sqrt(liopara_.max_points_per_voxel));
  double range_uncertanty = square(0.05);
  // double max_correspondence_distance = 1.0;
  double max_correspondence_distance =
      6 * std::sqrt(uncertanty_motion + map_uncertanty + range_uncertanty);

  Eigen::Vector15d last_dx = Eigen::Vector15d::Zero();
  double weight = 1000;
  Eigen::Matrix3d R_inv = Eigen::Matrix3d::Identity() * weight;
  int j = 0;
  const auto &cur_pose = bodystate_cur_.pose;

  Eigen::Matrix15d KH;
  for (j = 0; j < liopara_.max_iteration; ++j) {
    points_w = source;

    TransformPoints(cur_pose.matrix(), points_w);

    const auto &[src, tgt] =
        lio_map_.GetCorrespondences(points_w, max_correspondence_distance);

    auto compute_jacobian_and_residual = [&](auto i) {
      const Eigen::Vector3d &source_pt = src[i];
      const Eigen::Vector3d &target_pt = tgt[i];
      ;
      const Eigen::Vector3d residual = (source_pt - target_pt);
      Eigen::Vector3d R_bG_p = source_pt - cur_pose.translation();
      Eigen::Matrix3_15d H = Eigen::Matrix3_15d::Zero();
      H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      H.block<3, 3>(0, 6) = Sophus::SO3d::hat(R_bG_p);
      return std::make_tuple(H, residual);
    };

    const auto [HTRH, HTRz] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<size_t>{0, src.size()},
        // Identity
        ResultTuple(),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
          auto &[HTRH_private, HTRz_private] = J;
          for (auto i = r.begin(); i < r.end(); ++i) {
            const auto &[H, z] = compute_jacobian_and_residual(i);
            HTRH_private.noalias() += H.transpose() * R_inv * H;
            HTRz_private.noalias() += H.transpose() * R_inv * z;
          }
          return J;
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple {
          return a + b;
        });

    Eigen::Matrix15d S_inv = (HTRH + Cov_.inverse()).inverse();
    delta_x_ = S_inv * HTRz;
    KH = S_inv * HTRH;
    stateFeedback();

    if ((delta_x_ - last_dx).norm() < 0.001) {
      break;
    }
    last_dx = delta_x_;
    delta_x_.setZero();
  }
  Cov_ -= KH * Cov_;

  Sophus::SE3d pose_in_lidar_frame =
      bodystate_cur_.pose * Sophus::SE3d(liopara_.Trans_lidar_imu);
  Imu_Prediction_Covariance_.setZero();
  lio_map_.Update(frame_downsample, pose_in_lidar_frame);
  last_update_t_ = lidar_t_;
}

void LIOEKF::resetCov(Eigen::Matrix15d &Cov) {

  Cov.setZero();
  ImuError imuerror_std = liopara_.initstate_std.imuerror;

  Cov.block(POS_ID, POS_ID, 3, 3) =
      create_covariance_block(liopara_.initstate_std.pos);
  Cov.block(VEL_ID, VEL_ID, 3, 3) =
      create_covariance_block(liopara_.initstate_std.vel);
  Cov.block(ATT_ID, ATT_ID, 3, 3) =
      create_covariance_block(liopara_.initstate_std.euler);
  Cov.block(GYRO_BIAS_ID, GYRO_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrbias);
  Cov.block(ACC_BIAS_ID, ACC_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.accbias);
}

int LIOEKF::isToUpdate(double imutime1, double imutime2,
                       double updatetime) const {

  if (abs(imutime1 - updatetime) < TIME_ALIGN_ERR) {

    // updatetime is near to imutime1
    return 1;
  } else if (abs(imutime2 - updatetime) <= TIME_ALIGN_ERR) {

    // updatetime is near to imutime2
    return 2;
  } else if (imutime1 < updatetime && updatetime < imutime2) {

    // updatetime is between imutime1 and imutime2, but not near to either
    return 3;
  } else {

    // updatetime is not bewteen imutime1 and imutime2, and not near to either.
    return 0;
  }
}

void LIOEKF::ekfPredict(Eigen::Matrix15d &State_Transition_Mat,
                        Eigen::Matrix15d &Process_Noise_Cov) {

  assert(State_Transition_Mat.rows() == Cov_.rows());
  assert(Process_Noise_Cov.rows() == Cov_.rows());
  auto propagate = [&](const Eigen::Matrix15d &Sigma) {
    return State_Transition_Mat * Sigma * State_Transition_Mat.transpose() +
           Process_Noise_Cov;
  };
  // propagate system covariance and error state
  Cov_ = State_Transition_Mat * Cov_ * State_Transition_Mat.transpose() +
         Process_Noise_Cov;
  Imu_Prediction_Covariance_ =
      State_Transition_Mat * Imu_Prediction_Covariance_ *
          State_Transition_Mat.transpose() +
      Process_Noise_Cov + 1e-8 * Eigen::Matrix15d::Identity();
  delta_x_ = State_Transition_Mat * delta_x_;
}

void LIOEKF::ekfUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H,
                       Eigen::MatrixXd &R) {
  // debug_<<"ekfUpdate start"<<std::endl;
  assert(H.cols() == Cov_.rows());
  assert(dz.rows() == H.rows());
  assert(dz.rows() == R.rows());
  assert(dz.cols() == 1);

  // Compute Kalman Gain
  auto temp = H * Cov_ * H.transpose() + R;
  Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

  // update system error state and covariance
  Eigen::Matrix15d I;
  I.setIdentity();
  I = I - K * H;

  delta_x_ = delta_x_ + K * (dz - H * delta_x_);
  Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
}

void LIOEKF::stateFeedback() {
  // position error feedback
  Eigen::Vector3d delta_translation = delta_x_.block(POS_ID, 0, 3, 1);
  Eigen::Vector3d delta_quat = delta_x_.block(ATT_ID, 0, 3, 1);
  bodystate_cur_.pose.so3() =
      Sophus::SO3d::exp(delta_quat) * bodystate_cur_.pose.so3();
  bodystate_cur_.pose.translation() =
      bodystate_cur_.pose.translation() - delta_translation;
  // velocity error feedback
  Eigen::Vector3d delta_vel = delta_x_.block(VEL_ID, 0, 3, 1);
  bodystate_cur_.vel -= delta_vel;
  // IMU bias error feedback
  Eigen::Vector3d delta_bias_gyro = delta_x_.block(GYRO_BIAS_ID, 0, 3, 1);
  imuerror_.gyrbias += delta_bias_gyro;
  Eigen::Vector3d delta_bias_acc = delta_x_.block(ACC_BIAS_ID, 0, 3, 1);
  imuerror_.accbias += delta_bias_acc;
}

NavState LIOEKF::getNavState() {
  NavState state;
  state.pos = bodystate_cur_.pose.translation();
  state.vel = bodystate_cur_.vel;
  state.euler = Rotation::matrix2euler(bodystate_cur_.pose.rotationMatrix());
  state.imuerror = imuerror_;
  return state;
}

std::pair<Vector3dVector, Vector3dVector>
LIOEKF::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
  const auto voxel_size = liopara_.voxel_size;
  const auto frame_downsample =
      kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
  const auto source =
      kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
  return {source, frame_downsample};
}

Eigen::Matrix4d LIOEKF::poseTran(const Eigen::Matrix4d pose1,
                                 const Eigen::Matrix4d pose2) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R1, R2;
  Eigen::Vector3d T1, T2;

  R1 = pose1.block<3, 3>(0, 0);
  T1 = pose1.block<3, 1>(0, 3);
  R2 = pose2.block<3, 3>(0, 0);
  T2 = pose2.block<3, 1>(0, 3);

  pose.block<3, 3>(0, 0) = R1 * R2;
  pose.block<3, 1>(0, 3) = T1 + R1 * T2;

  return pose;
}

void LIOEKF::openResults() {
  std::string odomoutputpath = "ohm/data/pin_lio_ekf_odo.txt";

  odomRes_.open(odomoutputpath.c_str());
  odomRes_.setf(std::ios::fixed, std::ios::floatfield);
  odomRes_.precision(10);

  // std::string odomoutputpath = "ohm/data/lio_ekf_update.txt";

  // odomRes_.open(odomoutputpath.c_str());
  // odomRes_.setf(std::ios::fixed, std::ios::floatfield);
  // odomRes_.precision(10);
};

void LIOEKF::writeResults() {
  lio_ekf::NavState navstate = getNavState();

  Eigen::MatrixXd curCov = getCovariance();

  const auto rotmat_imu = liopara_.imu_tran_R.inverse();

  Eigen::Vector3d pos = rotmat_imu * navstate.pos;

  // transform pose from the used imu frame (front-right-down) to the original
  // imu frame
  Eigen::Matrix3d rotmat = liopara_.imu_tran_R.inverse() *
                           Rotation::euler2matrix(navstate.euler) *
                           liopara_.imu_tran_R;
  Eigen::Vector3d euler = Rotation::matrix2euler(rotmat);

  odomRes_ << std::setprecision(10) << getImutimestamp() << " "
           << pos.transpose() << " " << navstate.vel.transpose() << " "
           << euler.transpose() * R2D << " "
           << navstate.imuerror.gyrbias.transpose() << " "
           << navstate.imuerror.accbias.transpose() << std::endl;

  // Eigen::Quaterniond quat = Rotation::euler2quaternion(euler);

  // odomRes_tum_ << std::setprecision(18) << (lio_ekf_.getImutimestamp() /
  // 1e9)
  //              << "e+09"
  //              << " " << std::setprecision(5) << pos[0] << " " << pos[1] <<
  //              "
  //              "
  //              << pos[2] << " " << quat.x() << " " << quat.y() << " "
  //              << quat.z() << " " << quat.w() << std::endl;
}

void LIOEKF::publishMsgs() {
  // front-right-down to right-front-up for visualization
  Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
  tmp.block<3, 3>(0, 0) = liopara_.imu_tran_R.inverse();

  lio_ekf::NavState navstate = getNavState();

  Eigen::Matrix3d rotM = Rotation::euler2matrix(navstate.euler);
  Eigen::Matrix4d curpose = Eigen::Matrix4d::Identity();

  curpose.block<3, 3>(0, 0) = rotM;
  curpose.block<3, 1>(0, 3) = navstate.pos;

  newpose = poseTran(tmp, curpose);
  newpose = poseTran(newpose, tmp);

  std::vector<Eigen::Vector3d> tmpmap = LocalMap();
  TransformPoints(tmp, tmpmap);
}

} // namespace lio_ekf
