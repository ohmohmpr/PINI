import numpy as np
import matplotlib.pyplot as plt

def find_time(pred_data_LIO_EKF):
  start_time_idx = 100 * 10
  end_time_idx = - (40) * 10 # pred_data_LIO_EKF.shape[1]
  # start_time_idx = 0
  # end_time_idx = -1 

  return start_time_idx, end_time_idx



def plot_everything(pini_data, lio_ekf_data, title, start_time_idx, end_time_idx):

  pred_idx = np.where(pini_data[2] == -1) # predict
  delta_idx = np.where(pini_data[2] == 0)  # delta_x
  cov_idx = np.where(pini_data[2] == 10)  # cov
  n_idx = np.where(pini_data[2] == 15)  # n
  # update_idx = np.where(pini_data[2] == 1) # update
  
  pred_lio_ekf_idx = np.where(lio_ekf_data[2] == -1) # predict
  delta_lio_ekf_idx = np.where(lio_ekf_data[2] == 0) # delta_x
  cov_lio_elf_idx = np.where(lio_ekf_data[2] == 10)  # cov
  n_lio_elf_idx = np.where(lio_ekf_data[2] == 15)  # n
  point_diff_idx = np.where(lio_ekf_data[2] == 11)  # point_diff
  # update_lio_ekf_idx = np.where(lio_ekf_data[2] == 1) # update
  
  pred_pini_data = np.reshape(pini_data[:, pred_idx], (pini_data.shape[0], -1))
  delta_x_pini_data = np.reshape(pini_data[:, delta_idx], (pini_data.shape[0], -1))
  cov_pini_data = np.reshape(pini_data[:, cov_idx], (pini_data.shape[0], -1))
  n_pini_data = np.reshape(pini_data[:, n_idx], (pini_data.shape[0], -1))
  
  pred_lio_ekf_data = np.reshape(lio_ekf_data[:, pred_lio_ekf_idx], (lio_ekf_data.shape[0], -1))
  delta_x_lio_ekf_data = np.reshape(lio_ekf_data[:, delta_lio_ekf_idx], (lio_ekf_data.shape[0], -1))
  cov_lio_ekf_data = np.reshape(lio_ekf_data[:, cov_lio_elf_idx], (lio_ekf_data.shape[0], -1))
  n_lio_ekf_data = np.reshape(lio_ekf_data[:, n_lio_elf_idx], (lio_ekf_data.shape[0], -1))
  point_diff_lio_ekf_data = np.reshape(lio_ekf_data[:, point_diff_idx], (lio_ekf_data.shape[0], -1))

  plot_delta_x(pred_lio_ekf_data, None, pred_pini_data, None, title, start_time_idx, end_time_idx, 'state', point_diff_lio_ekf_data)
  plot_delta_x(delta_x_lio_ekf_data, None, delta_x_pini_data, None, title, start_time_idx, end_time_idx, 'delta_x', point_diff_lio_ekf_data)
  plot_delta_x(cov_lio_ekf_data, None, cov_pini_data, None, title, start_time_idx, end_time_idx, 'cov', point_diff_lio_ekf_data)
  plot_delta_x(n_lio_ekf_data, None, n_pini_data, None, title, start_time_idx, end_time_idx, 'N', point_diff_lio_ekf_data)




def plot_delta_x(pred_data_LIO_EKF, update_data_LIO_EKF, pred_data_PINI, update_data_PINI, title, start_time_idx, end_time_idx, del_name, point_diff):

  if title == 'translation':
    label_idx = 3
  elif title == 'velocity':
    label_idx = 6
  elif title == 'rotation':
    label_idx = 9
  elif title == 'gyro_bias':
    label_idx = 12
  elif title == 'acc_bias':
    label_idx = 15

  figure, axis = plt.subplots(4, 1, sharex=True, figsize=(12, 9))

  axis[0].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx  , start_time_idx:end_time_idx], 'r-', label='x_PINI') # x
  axis[1].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx+1, start_time_idx:end_time_idx], 'g-', label='y_PINI') # y
  axis[2].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx+2, start_time_idx:end_time_idx], 'b-', label='z_PINI') # z

  axis[0].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx  , start_time_idx:end_time_idx], 'k--', label='x_LIO_EKF') # x
  axis[1].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx+1, start_time_idx:end_time_idx], 'k--', label='y_LIO_EKF') # y
  axis[2].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx+2, start_time_idx:end_time_idx], 'k--', label='z_LIO_EKF') # z

  axis[3].plot(point_diff[0, start_time_idx:end_time_idx] - point_diff[0,0], point_diff[3, start_time_idx:end_time_idx], 'k--', label='point_diff') # z

  axis[0].set_title("X")
  axis[1].set_title("Y")
  axis[2].set_title("Z")
  axis[0].legend()
  axis[1].legend()
  axis[2].legend()
  figure.supxlabel('time (second)')
  figure.supylabel(title)
  figure.suptitle(f'{title, del_name}', fontsize=12)

  plt.plot()
  plt.show()