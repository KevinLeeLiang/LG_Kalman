# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import matplotlib.pyplot as plt
import numpy as np
import sys
from CsvReader import CsvFileReader
from FirstOriderKalmanFilter import KalmanFilter_1stOrderSys
from SecondOrderKalmanFilter import KalmanFilter_2ndOrderSys


def print_hi(name):
  # Use a breakpoint in the code line below to debug your script.
  print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def CalFk(t):
  return np.array([[1, t, 0.5*t**2], [0, 1, t], [0, 0, 1]])

def wgn(x, snr):
  snr = 10**(snr/10.0)
  xpower = np.sum(x**2)/len(x)
  npower = xpower / snr
  return np.random.randn(len(x)) * np.sqrt(npower)

def LG_Kalman_Filter(T, delt_t, Hk, Pk_1, xk_1, Qvk, zk):
  res_kalman = []
  res_kalman.append(xk_1[0])
  res_T = []
  res_T.append(T)
  for i in range(1, 11):
    Fk     = CalFk(T)
    Pk_k_1 = np.matmul(np.matmul(Fk, Pk_1), Fk.transpose())
    xk_k_1 = np.matmul(Fk, xk_1)
    tmp1   = np.matmul(Pk_k_1, Hk.transpose())
    tmp2   = np.matmul(np.matmul(Hk, Pk_k_1), Hk.transpose())
    Gk_k_1 = tmp1*(tmp2 + Qvk)**(-1)
    xk_1   = xk_k_1 + Gk_k_1*(zk[i] - np.matmul(Hk, xk_k_1))
    res_kalman.append(xk_1[0])
    T = T + delt_t
    res_T.append(T)
    
    return res_T, res_kalman

def LG_Kalman_Filter_Test1():
  '''
  状态转移方程
  xk_1 = Fk*xk + wk, Fk = [rk, vk, ak]T
  zk   = Hk*xk + vk
  其中
  Fk = [[1, T, T^2], [0, 1, T], [0, 0, 1]]
  Hk = [1, 0, 0] 速度和加速度不能观
  E(r0)=0, sigma(r0)^2=8; E(v0)=0, sigma(v0)^2=10; E(a0)=0, sigma(a0)^2=5
  观测噪声为N(0, sigma(n)^2)
  '''
  '''
  状态计算分成——时间更新和滤波更新
  时间更新：^xk|k-1 = Fk-1*^xk=1
  滤波更新：^xk     = ^xk|k-1 + Gk|k-1x~zk|k-1
          Gk|k-1  = Pk|k-1HTk[HkPk|k-1HTk + Qvk]^-1
  方差计算分成——时间更新和滤波更新
  时间更新：Pk|k-1  = Fk-1Pk-1FTk-1
  滤波更新：Pk      = (I - Gk|k-1Hk)Pk|k-1
  '''
  T = 2.0
  # Fk = np.arange(9).reshape(3, 3)
  #
  Fk = np.array([[1, T, T**2], [0, 1, T], [0, 0, 1]])
  delt_t = 2.0
  # Hk = np.arange(3).reshape(3, 1)
  # Hk[0] = 1
  # Hk[1] = 0
  # Hk[2] = 0
  Hk = np.array([1., 0., 0.])
  pk_1 = np.array([[8., 0., 0.], [0., 10., 0.], [0., 0., 5.]])
  xk = np.array([0., 0., 0.2])
  Qvk = 0.15
  zk = np.array([0, 0.36, 1.56, 3.64, 6.44, 10.5, 14.8, 20.0, 25.2, 32.2, 40.4])
  T_res, kalman_res = LG_Kalman_Filter(T, delt_t, Hk, pk_1, xk, Qvk, zk)
  plt.plot(T_res, kalman_res, "b-")
  plt.plot(T_res, zk, "r-")
  plt.show()

def LG_Kalman_Filter_Test2():
  T = 2.0
  xplot, yplot= [], []
  Fk = np.array([[1., T, T**2, 0., 0.], [0., 1., T, 0., 0.], [0., 0., 1., 0., 0.], [0., 0., 0., 1., T], [0., 0., 0., 0., 1.]])
  Hk = np.array([[1., 0., 0., 0, 0], [0., 0., 0., 1., 0.]])
  sigma = 10000
  t1 = 400
  t2 = 600
  t3 = 610
  t4 = 660
  totaltime = 700
  vy = -12
  ax = 0
  vx = 0
  xadd = 0
  # 初始值
  xk1 = np.array([1000, 0, ax, 800, -12])
  zk  = np.array([[1000], [800]])
  pk1 = np.diag([sigma, sigma/T, sigma/T**2, sigma, sigma/T])
  Qvk = sigma*np.eye(2)
    
  for i in range(1, 450):
    if i > 200 and i <= 300:
      ax = 0.075
      xk1[3][1] = ax
    if i > 300 and i <= 305:
      ax = 0
      xk1[3][1] = ax
    if i > 305 and i <= 330:
      ax = -0.3
      xk1[3][1] = ax
    if i > 330:
      ax = 0
      xk1[3][1] = ax
  
    xadd = xadd + vx*T+0.5*ax*T**2
    vx = vx + ax*T
    zk = np.array([1000 + xadd + wgn([1], 40), 800 + vy*T*(i - 1) + wgn([1], 40)])
    xk2 = Fk*xk1
    pk2 = Fk*pk1*Fk.transpose()
    Gk = pk2*Hk.transpose()*np.linalg.inv(Hk*pk2*Hk.transpose() + Qvk)
    pk3 = (np.eye(5) - Gk*Hk)*pk2
    xk3 = xk2 + Gk*(zk - Hk*xk2)
    pk1 = pk3
    xk1 = xk3
    xplot.append(xk3)
    yplot.append(zk)
    
    tst = 0

def LG_Kalman_Filter_Test3():
  dir = "test.csv"
  reader = CsvFileReader()
  reader.CsvFileReader(dir)
  vel_data = reader.GetVel_Origin()
  kalman_filter = KalmanFilter_1stOrderSys(0.001, 0.1)
  vel_filters = []
  i = 0
  xaxis = []
  for vel in vel_data:
    vel_filter = kalman_filter.KalmanFilterCalc(vel)
    vel_filters.append(vel_filter)
    xaxis.append(i)
    i += 1
  plt.plot(xaxis, vel_filters, "g-")
  plt.plot(xaxis, vel_data, "b-")
  plt.show()
  return

def LG_Kalman_Filter_Test4():
  dir = "test.csv"
  reader = CsvFileReader()
  reader.CsvFileReader(dir)
  vel_data = reader.GetVel_Origin()
  acc_data = reader.GetAcc_Origin()
  kalman_filter = KalmanFilter_2ndOrderSys()
  kalman_filter1 = KalmanFilter_1stOrderSys(0.001, 0.1)
  vel_filters = []
  vel_filters1 = []
  i = 0
  xaxis = []
  for i in range(0, len(vel_data)):
    vel, acc = vel_data[i], acc_data[i]
    vel_filters1.append(kalman_filter1.KalmanFilterCalc(vel))
    if i == 0:
      kalman_filter.SetState0(vel, acc)
      vel_filter = vel
    else:
      vel_filter = kalman_filter.Horizon_klm_estimate(vel, acc, 0.05)
    vel_filters.append(vel_filter)
    xaxis.append(i)
    
  plt.plot(xaxis, vel_filters, "g-")
  plt.plot(xaxis, vel_data, "b-")
  plt.plot(xaxis, vel_filters1, "r-")
  plt.show()
  return

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
  print_hi('PyCharm')
  # LG_Kalman_Filter_Test1()
  # LG_Kalman_Filter_Test2()
  LG_Kalman_Filter_Test4()
    
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
