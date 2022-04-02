import numpy as np
class KalmanInfo:
	def __init__(self):
		self.x_state = np.mat(np.arange(2).reshape(2, 1), float)
		self.y_oberve = np.mat(np.arange(2).reshape(2, 1), float)
		self.A_matrix = np.mat(np.arange(4).reshape(2, 2), float)
		self.p_variance = np.mat(np.arange(4).reshape(2, 2), float)
		self.K_gain = np.mat(np.arange(4).reshape(2, 2), float)
		self.R_variance = np.mat(np.arange(4).reshape(2, 2), float)
		self.Q_variance = np.mat(np.arange(4).reshape(2, 2), float)

class KalmanFilter_2ndOrderSys:
	def __init__(self):
		self.__kalman_info = KalmanInfo()
		self.__kalman_info.A_matrix[0, 0] = 1
		self.__kalman_info.A_matrix[1, 1] = 1
		self.__kalman_info.p_variance[0, 0] = 1
		self.__kalman_info.p_variance[1, 1] = 1
		self.__kalman_info.R_variance[0, 0] = 0.01
		self.__kalman_info.R_variance[1, 1] = 0.01
		self.__kalman_info.Q_variance[0, 0] = 0.001
		self.__kalman_info.Q_variance[1, 1] = 0.001
	
	def SetState0(self, v, a):
		self.__kalman_info.x_state[0, 0] = v
		self.__kalman_info.x_state[1, 0] = a
	
	def Horizon_klm_estimate(self, v, a, delt_t):
		self.__kalman_info.A_matrix[0, 1] = delt_t
		self.__kalman_info.y_oberve[0, 0] = v
		self.__kalman_info.y_oberve[1, 0] = a
		# step 1 predict x
		self.__kalman_info.x_state[0, 0] = self.__kalman_info.x_state[0, 0] + delt_t * self.__kalman_info.x_state[1, 0]
		# step 2 predict p
		A_T_Matrix = self.__kalman_info.A_matrix.transpose()
		p_temp = np.dot(self.__kalman_info.A_matrix, self.__kalman_info.p_variance)
		p_temp2 = np.dot(p_temp, A_T_Matrix)
		self.__kalman_info.p_variance = p_temp2 + self.__kalman_info.Q_variance
		# step 3 calculate K
		k_temp = self.__kalman_info.p_variance + self.__kalman_info.R_variance
		matrix3_i = np.linalg.inv(k_temp)
		self.__kalman_info.K_gain = np.dot(self.__kalman_info.p_variance, matrix3_i)
		z_delta = np.mat([[self.__kalman_info.x_state[0, 0] - self.__kalman_info.y_oberve[0, 0]],
												[self.__kalman_info.x_state[1, 0] - self.__kalman_info.y_oberve[1, 0]]])
		statue_corrent = np.dot(self.__kalman_info.K_gain, z_delta)
		self.__kalman_info.x_state = self.__kalman_info.x_state - statue_corrent
		# step 5 update P
		P_temp = np.eye(2, 2)
		P_temp = P_temp - self.__kalman_info.K_gain
		P_update = np.dot(P_temp, self.__kalman_info.p_variance)
		self.__kalman_info.p_variance = P_update
		
		return self.__kalman_info.x_state[0, 0]
