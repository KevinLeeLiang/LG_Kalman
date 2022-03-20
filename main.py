# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import matplotlib.pyplot as plt
import numpy as np
def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def CalFk(t):
    return np.array([[1, t, 0.5*t**2], [0, 1, t], [0, 0, 1]])

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


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
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
    
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
