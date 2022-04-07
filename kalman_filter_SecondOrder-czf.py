# Author: chenzf2013@163.com
# Creatine: 2022/3/26/026
# Filename: kalman_filter_SecondOrder
# Description: simple introduction of the code
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class KalmanFilter2Order:
    """
    二阶线性卡尔曼滤波器，处理加速度、速度；
    """

    def __init__(self, size=100, sampleTime=0.05, q11=0.001, q12=0.0, q21=0.0, q22=0.1, r11=0.001, r12=0.0, r21=0.0,
                 r22=0.1):
        #  参数: 噪声方差矩阵——可调参数 #########################################################################
        self.T = sampleTime
        self.A = np.mat([[1, sampleTime], [0, 1]])  # 状态矩阵,
        self.H = np.mat([[1, 0], [0, 1]])  # 观测矩阵,
        self.Q = np.mat([[q11, q12], [q21, q22]])  # 过程噪声的协方差矩阵：两个状态值x=[x1 ; x2]，两个状态值的过程噪声分别为w=[w1; w2];
        #  过程噪声符合高斯分布p(w1)~(0,q11), p(w2)~(0,q22); 两个状态值的过程噪声组成的协方差矩阵为Q,  p(w)~(0,Q)；Q=[q11,q12; q21,q22 ]
        self.R = np.mat([[r11, r12], [r21, r22]])  # 测量噪声的协方差矩阵；两个传感器的测量值z=[z1 ; z2]，两个传感器的测量噪声分别为v=[v1; v2];
        #  测量噪声符合高斯分布p(v1)~(0,r11), p(v2)~(0,r22); 两个状态值的测量噪声组成的协方差矩阵为R,  p(v)~(0,R)；R=[r11,r12; r21,r22 ]
        self.Ieye = np.mat([[1, 0], [0, 1]])
        #  2I2O-初始化各矩阵 #########################################################################
        self.filterValue = np.mat([0, 0]).T  # 校正：后验估计值（卡尔曼滤波结果）；2x1
        self.kalmanGain = np.mat([[1, 0], [0, 1]])  # 校正：卡尔曼增益；
        self.P = np.mat([[1, 0], [0, 1]])  # 校正：更新误差协方差；初始值不重要，都赋值为1就可以
        self.P_pre = np.mat([[1, 0], [0, 1]])  # 预测：先验误差协方差值；初始值不重要，都赋值为1就可以

        #  其它变量 #########################################################################
        self.measurementLast = np.mat([0, 0]).T  # 校正：后验估计值（卡尔曼滤波结果）；2x1
        self.measurementNew = np.mat([0, 0]).T  # 校正：后验估计值（卡尔曼滤波结果）；2x1
        self.Counter = 0
        self.speedKF = np.empty([size, 1])
        self.accKF = np.empty([size, 1])
        self.accModfiy = np.empty([size, 1])
        self.dataSize = size

    def kalmanFilterCalc(self, new_speed=np.zeros([3]), new_acc=np.zeros([3])):
        print("##log## 开始kalmanFilterCalc ：")

        for i in range(self.dataSize):  # self.dataSize =5;    0,1,2,3,4
            if 0 == i:  # 第一次进来初始化
                speed1 = float(new_speed[0])
                acc1 = float(new_acc[0])
                self.filterValue = np.mat([[speed1], [acc1]])
                self.speedKF[0, 0] = speed1
                self.accKF[0, 0] = acc1
                self.accModfiy[0, 0] = acc1
                continue  # 语句被用来告诉Python跳过当前循环块中的剩余语句，然后继续进行下一轮循环。
            speedModify = float(new_speed[i])
            accModify = float(new_acc[i])
            # 做imu加速度限幅滤波:加速度增长限幅 0.15*g ###########################################
            limitAccDelta = 0.15 * 9.8
            limitAccDelta_nav = -0.15 * 9.8
            acc_KF_last = float(self.filterValue.A[1, 0])
            acc_delta = accModify - acc_KF_last
            if acc_delta > limitAccDelta:
                accModify = acc_KF_last + limitAccDelta
            elif acc_delta < limitAccDelta_nav:
                accModify = acc_KF_last + limitAccDelta
            self.measurementNew = np.mat([[speedModify], [accModify]])

            # %%%%%%%%%%%%%%%%%%% 卡尔曼滤波过程 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            predictValue = self.A * self.filterValue  # 1)预测：先验状态估计值，依据先验公式得到的估计值； （3.36)
            self.P_pre = self.A * self.P * self.A.T + self.Q  # 2)预测：（先验）预测协方差矩阵 （3.39），此处应该是A*p*A^T+Q； 一维kalman不影响A=A^T
            self.kalmanGain = self.P_pre * self.H.T * (
                        self.H * self.P_pre * self.H.T + self.R).getI()  # 3)校正：计算Kalman增益矩阵； （3.38）
            # 正确写法：KalmanGain(k+1) = P_covariance_pre(k+1)* H' * inv(H*P_covariance_pre(k+1)* H'+R);
            # %卡尔曼滤波器核心公式：当前的估计值(滤波结果)=上一次的估计值+卡尔曼增益系数 * (当前的测量值 - 上一次的估计值)
            self.filterValue = predictValue + self.kalmanGain * (
                    self.measurementNew - self.H * predictValue)  # 4)校正：后验状态估计 更新 ； （3.37）
            self.P = (self.Ieye - self.kalmanGain * self.H) * self.P_pre  # %5)校正：误差协方差矩阵 更新； （3.40)
            self.speedKF[i, 0] = self.filterValue.A[0, 0]
            self.accKF[i, 0] = self.filterValue.A[1, 0]
            self.accModfiy[i, 0] = accModify
        print("##log## 结束kalmanFilterCalc ：")

        return self.speedKF*3.6, self.accKF, self.accModfiy

def Draw(dir):
    file_brake_thr = dir
    temp_path_new = os.path.dirname(file_brake_thr)
    temp_df = pd.read_csv(file_brake_thr, encoding="utf-8-sig")

    #  ---------- 1.列数据精简： 提取有用数据，去处多余的数据列 ----------
    #  列切片,用标签选择提取用到的多列数据：
    useful_column_data = temp_df.loc[:, ['ab_time', 'gps_speed', 'acceleration', 'vehicle_speed', 'throttle_percentage',
                                         'brake_percentage', 'steering_angle', 'gear_location', 'drive_mode', 'speed_KF',
                                         'acc_KF', 'accModfiy']]
    data_spdGPS = np.array(useful_column_data['gps_speed'])
    data_spdKF  = np.array(useful_column_data['speed_KF'])
    data_spdT   = np.linspace(0, len(data_spdKF), len(data_spdKF))
    plt.plot(data_spdT, data_spdGPS, "r-")
    plt.plot(data_spdT, data_spdKF, "b-")
    plt.show()
    
if __name__ == '__main__':
    """测试代码"""
    """
       对转换编码并合并后的单个csv文件进行数据处理
       :param file_source:待处理数据 输入的文件完整路径; 格式举例 ...\\brake_thr.csv
       :return:
       @note 可以考虑另一种途径： 直接由GPS速度来进行滤波求解速度值+加速度值 ，但是其实实际控制系统还是使用车控程序滤波后的算法来求解；
   """
    # 读取数据
    # file_brake_thr = 'D:\\BUAA_PhD\A1_Project\\20211214_kalman_filter\\kalman_filter_linear_czf\\DataSet\\ccu_20210929_152844_152844.csv_modify.csv'
    file_brake_thr = 'brake_thr.csv'
    temp_path_new = os.path.dirname(file_brake_thr)
    temp_df = pd.read_csv(file_brake_thr, encoding="utf-8-sig")

    #  ---------- 1.列数据精简： 提取有用数据，去处多余的数据列 ----------
    #  列切片,用标签选择提取用到的多列数据：
    useful_column_data = temp_df.loc[:, ['绝对时间', 'gps速度', '惯导纵向加速度', '实际车速', '油门控制量', '电制动控制量', '前轮转角', '档位', '驾驶模式']]
    useful_column_data = useful_column_data.rename(
        columns={'绝对时间': 'ab_time', 'gps速度': 'gps_speed', '惯导纵向加速度': 'acceleration',
                 '实际车速': 'vehicle_speed', '油门控制量': 'throttle_percentage',
                 '电制动控制量': 'brake_percentage', '前轮转角': 'steering_angle',
                 '档位': 'gear_location', '驾驶模式': 'drive_mode'})  # 修改列名为英文
    file_brake_thr_simple = os.path.join(temp_path_new, "brake_thr_simple.csv")
    Num = useful_column_data.__len__()
    KF_2Order_1 = KalmanFilter2Order(size=Num, q11=0.002, q12=0.00037, q21=-0.00037, q22=0.01,
                                     r11=0.01, r12=0.0, r21=0.0, r22=1.4)

    data_speed = np.array(useful_column_data['gps_speed']/3.6)
    data_acc = np.array(useful_column_data['acceleration']*9.8)
    useful_column_data['speed_KF'], useful_column_data['acc_KF'], useful_column_data['accModfiy']= KF_2Order_1.kalmanFilterCalc(data_speed, data_acc)

    useful_column_data.to_csv(path_or_buf=file_brake_thr_simple, index=False,
                              encoding="utf-8-sig")  # 列数据精简后的文件，保存为新的brake_thr_simple.csv文件
    print("##log## kalmanFilter结果.csv存放目录: ", str(file_brake_thr_simple))
    Draw("brake_thr_simple.csv")