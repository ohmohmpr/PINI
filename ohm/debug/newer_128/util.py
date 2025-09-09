import numpy as np
import matplotlib.pyplot as plt

def find_time(pred_data_LIO_EKF):
  start_time_idx = 100 * 10
  end_time_idx = - (40) * 10 # pred_data_LIO_EKF.shape[1]
  # start_time_idx = 0
  # end_time_idx = -1 

  return start_time_idx, end_time_idx



def plot_state(pred_data_LIO_EKF, update_data_LIO_EKF, pred_data_PINI, update_data_PINI, title, start_time_idx, end_time_idx):

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

  figure, axis = plt.subplots(3, 1, sharex=True, sharey=True, figsize=(12, 9))
  
  axis[0].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx  , start_time_idx:end_time_idx], 'r--', label='x_LIO_EKF') # x
  axis[1].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx+1, start_time_idx:end_time_idx], 'g--', label='y_LIO_EKF') # y
  axis[2].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx+2, start_time_idx:end_time_idx], 'b--', label='z_LIO_EKF') # z

  axis[0].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx  , start_time_idx:end_time_idx], 'r-', label='x_PINI') # x
  axis[1].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx+1, start_time_idx:end_time_idx], 'g-', label='y_PINI') # y
  axis[2].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx+2, start_time_idx:end_time_idx], 'b-', label='z_PINI') # z

  axis[0].set_title("X")
  axis[1].set_title("Y")
  axis[2].set_title("Z")
  axis[0].legend()
  axis[1].legend()
  axis[2].legend()
  figure.supxlabel('time (second)')
  figure.supylabel(title)
  figure.suptitle('State', fontsize=12)

  plt.show()

def plot_delta_x(pred_data_LIO_EKF, update_data_LIO_EKF, pred_data_PINI, update_data_PINI, title, start_time_idx, end_time_idx):

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

  figure, axis = plt.subplots(3, 1, sharex=True, sharey=True, figsize=(12, 9))
  
  axis[0].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx  , start_time_idx:end_time_idx], 'r--', label='x_LIO_EKF') # x
  axis[1].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx+1, start_time_idx:end_time_idx], 'g--', label='y_LIO_EKF') # y
  axis[2].plot(pred_data_LIO_EKF[0, start_time_idx:end_time_idx] - pred_data_LIO_EKF[0,0], pred_data_LIO_EKF[label_idx+2, start_time_idx:end_time_idx], 'b--', label='z_LIO_EKF') # z

  axis[0].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx  , start_time_idx:end_time_idx], 'r-', label='x_PINI') # x
  axis[1].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx+1, start_time_idx:end_time_idx], 'g-', label='y_PINI') # y
  axis[2].plot(pred_data_PINI[0, start_time_idx:end_time_idx] - pred_data_PINI[0,0], pred_data_PINI[label_idx+2, start_time_idx:end_time_idx], 'b-', label='z_PINI') # z

  axis[0].set_title("X")
  axis[1].set_title("Y")
  axis[2].set_title("Z")
  axis[0].legend()
  axis[1].legend()
  axis[2].legend()
  figure.supxlabel('time (second)')
  figure.supylabel(title)
  figure.suptitle('Delta_x', fontsize=12)
  plt.show()