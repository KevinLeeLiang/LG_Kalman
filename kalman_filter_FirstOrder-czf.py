# Author: chenzf2013@163.com
# Creatine: 2021/12/16/016
# Filename: kalman_filter
# Description: simple introduction of the code
import configparser
import os

import pandas as pd

from tage_merge_csv import get_csv_data


class KalmanFilter:
    """
    卡尔曼滤波
    """

    def __init__(self, q_val=0.001, r_val=0.15):
        self.filterValue = 0.0288  # 滤波后的值, 这个初值不重要, 因为会不停更新
        self.kalmanGain = 0  # Kalman增益
        self.A = 1  # 状态矩阵, 在一阶滤波器里面就是一个常数
        self.H = 1  # 观测矩阵, 在一阶滤波器里面就是一个常数
        self.Q = q_val  # 过程噪声的方差, 状态方程是理想的，其估计过程是存在过程噪声的,；是对预测模型（状态方程）的信任程度#
        self.R = r_val  # 测量噪声的方差, 表示传感器固有的误差, 是对测量系统的信任程度#
        self.P = 1  # 协方差, 这个初值不重要, 因为会不停更新
        self.P_pre = 1
        self.Ieye = 1

    def kalmanFilterCalc(self, new_value):
        predictValue = self.A * self.filterValue  # 1)预测：先验状态估计值，依据先验公式得到的估计值； （3.36)
        self.P_pre = self.A * self.P * self.A + self.Q  # 2)预测：（先验）预测协方差矩阵 （3.39），此处应该是A*p*A^T+Q； 一维kalman不影响A=A^T
        self.kalmanGain = self.P_pre * self.H / (self.H * self.P_pre * self.H + self.R)  # 3)校正：计算Kalman增益矩阵； （3.38）
        # 正确写法：KalmanGain(k+1) = P_covariance_pre(k+1)* H' * inv(H*P_covariance_pre(k+1)* H'+R);
        # %卡尔曼滤波器核心公式：当前的估计值(滤波结果)=上一次的估计值+卡尔曼增益系数 * (当前的测量值 - 上一次的估计值)
        self.filterValue = predictValue + self.kalmanGain * (
                new_value - self.H * predictValue)  # 4)校正：后验状态估计 更新 ； （3.37）
        self.P = (self.Ieye - self.kalmanGain * self.H) * self.P_pre  # %5)校正：误差协方差矩阵 更新； （3.40)

        return self.filterValue


if __name__ == '__main__':
    """测试代码"""

    """
       对转换编码并合并后的单个csv文件进行数据处理
       :param file_source:待处理数据 输入的文件完整路径; 格式举例 ...\\brake_thr.csv
       :return:
       @note 可以考虑另一种途径： 直接由GPS速度来进行滤波求解速度值+加速度值 ，但是其实实际控制系统还是使用车控程序滤波后的算法来求解；
   """
    # 读取数据
    # file_brake_thr = 'D:\\BUAA_PhD\\A1_Project\\20211214_KMF\\data_20210929\\ccu_20210929_151047_151047.csv_modify.csv'
    # temp_path_new = os.path.dirname(file_brake_thr)
    # temp_df = pd.read_csv(file_brake_thr, encoding="utf-8-sig")
    #
    # #  ---------- 1.列数据精简： 提取有用数据，去处多余的数据列 ----------
    # #  列切片,用标签选择提取用到的多列数据：
    # useful_column_data = temp_df.loc[:, ['绝对时间', 'gps速度', '惯导纵向加速度', '实际车速', '油门控制量', '电制动控制量', '前轮转角', '档位', '驾驶模式']]
    # useful_column_data = useful_column_data.rename(
    #     columns={'绝对时间': 'ab_time', 'gps速度': 'gps_speed', '惯导纵向加速度': 'acceleration',
    #              '实际车速': 'vehicle_speed', '油门控制量': 'throttle_percentage',
    #              '电制动控制量': 'brake_percentage', '前轮转角': 'steering_angle',
    #              '档位': 'gear_location', '驾驶模式': 'drive_mode'})  # 修改列名为英文
    # # file_brake_thr_simple = os.path.join(temp_path_new, "brake_thr_simple.csv")
    # useful_column_data.to_csv(path_or_buf=file_brake_thr_simple, index=False,
    #                           encoding="utf-8-sig")  # 列数据精简后的文件，保存为新的brake_thr_simple.csv文件
    # data_test = KalmanFilter(q_val=0.001, r_val=0.015)
