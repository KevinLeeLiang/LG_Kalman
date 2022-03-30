class KalmanInfo:
	def __init__(self):
		self.filter_val  = 0				# 滤波后的值,这个初值不重要,因为会不停更新
		self.kalman_gain = float		# Kalamn增益
		self.A           = 1				# 状态矩阵,在一阶滤波器里面就是一个常数
		self.H           = 1				# 观测矩阵,在一阶滤波器里面就是一个常数
		self.Q           = 0				# 状态矩阵的方差,表示预测模型的噪声,是对预测模型的信任程度
		self.R           = 0				# 观测矩阵的方差,表示测量模型的噪声,是对测量系统的信任程度
		self.P           = 1				# 状态协方差,这个初值不重要,因为会不停更新
		
class KalmanFilter:
	def __init__(self, Qval, Rval):
		self.__kalman_info = KalmanInfo()
		self.__kalman_info.Q  = Qval
		self.__kalman_info.R  = Rval
	
	def KalmanFilterCalc(self, new_value):
		pre_val = self.__kalman_info.A * self.__kalman_info.filter_val   # 计算预测值
		self.__kalman_info.P = self.__kalman_info.A*self.__kalman_info.A*self.__kalman_info.P + self.__kalman_info.Q # 求协方差
		self.__kalman_info.kalman_gain = \
			self.__kalman_info.P*self.__kalman_info.H/\
			(self.__kalman_info.P*self.__kalman_info.H*self.__kalman_info.H + self.__kalman_info.R) # 计算kalman增益
		self.__kalman_info.filter_val = pre_val + (new_value - pre_val)*self.__kalman_info.kalman_gain # 计算输出值
		self.__kalman_info.P = (1 - self.__kalman_info.kalman_gain*self.__kalman_info.H)*self.__kalman_info.P  # 更新协方差
		return self.__kalman_info.filter_val
		