#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
import matplotlib.pyplot as plt
#from scipy.stats import norm


#Python简单模拟预测小车位置和速度的一段程序，观测的值是一段从0到99，速度为1的匀速行驶路径
#，加入的噪声是均值为0，方差为1的高斯噪声。
 

# 创建一个0-99的一维矩阵

z = [i for i in range(100)]

z_watch = np.mat(z)

#print(z_mat)

 

# # 创建一个方差为1的高斯噪声，精确到小数点后两位


noise = np.round(np.random.normal(0, 1, 100), 2)

noise_mat = np.mat(noise)
#print(noise)



# # 将z的观测值和噪声相加

z_mat = z_watch + noise_mat

#print(z_mat)

 

# # 定义x的初始状态

x_mat = np.mat([[0,], [0,]])
#print(x_mat)
# # 定义初始状态协方差矩阵

p_mat = np.mat([[1, 0], [0, 1]])

# # 定义状态转移矩阵，因为每秒钟采一次样，所以delta_t = 1

f_mat = np.mat([[1, 1], [0, 1]])

# # 定义状态转移协方差矩阵，这里我们把协方差设置的很小，因为觉得状态转移矩阵准确度高

q_mat = np.mat([[0.0001, 0], [0, 0.0001]])

# # 定义观测矩阵

h_mat = np.mat([1, 0])

# # 定义观测噪声协方差

r_mat = np.mat([1])

 

for i in range(100):

    x_predict = f_mat * x_mat

    p_predict = f_mat * p_mat * f_mat.T + q_mat
    
    kalman = p_predict * h_mat.T / (h_mat * p_predict * h_mat.T + r_mat)
    
    x_mat = x_predict + kalman *(z_mat[0, i] - h_mat * x_predict)
    
    
    p_mat = (np.eye(2) - kalman * h_mat) * p_predict

    

    plt.plot(x_mat[0, 0], x_mat[1, 0], 'ro', markersize = 1)

    

plt.show()


#print("int")

# #x is pedestrian state
# x = np.matrix([[0.0, 0.0, 0.0, 0.0]]).T
# print(x, x.shape)

# #p is covariance matrix
# P = np.diag([1000.0, 1000.0, 1000.0, 1000.0])
# print(P, P.shape)

# dt = 0.1 # Time Step between Filter Steps

# #F is state matrix 
# F = np.matrix([[1.0, 0.0, dt, 0.0],
#               [0.0, 1.0, 0.0, dt],
#               [0.0, 0.0, 1.0, 0.0],
#               [0.0, 0.0, 0.0, 1.0]])
# print(F, F.shape)


# #H is measurement matrix
# H = np.matrix([[0.0, 0.0, 1.0, 0.0],
#               [0.0, 0.0, 0.0, 1.0]])
# print(H, H.shape)

# #计算测量过程的噪声的协方差矩阵R和处理噪声的协方差矩阵Q

# ra = 10.0**2

# R = np.matrix([[ra, 0.0],
#               [0.0, ra]])
# print(R, R.shape)


# ra = 0.09
# R = np.matrix([[ra, 0.0],
#               [0.0, ra]])
# print(R, R.shape)

# sv = 0.5
# G = np.matrix([[0.5*dt**2],
#                [0.5*dt**2],
#                [dt],
#                [dt]])
# Q = G*G.T*sv**2


# printing.init_printing()
# dts = Symbol('dt')
# Qs = Matrix([[0.5*dts**2],[0.5*dts**2],[dts],[dts]])
# #Qs*Qs.T

# I = np.eye(4)
# print(I, I.shape)

# m = 200 # Measurements
# vx= 20 # in X
# vy= 10 # in Y

# mx = np.array(vx+np.random.randn(m))
# my = np.array(vy+np.random.randn(m))
# measurements = np.vstack((mx,my))

# print(measurements.shape)
# print('Standard Deviation of Acceleration Measurements=%.2f' % np.std(mx))
# print('You assumed %.2f in R.' % R[0,0])

# fig = plt.figure(figsize=(16,5))
# plt.step(range(m),mx, label='$\dot x$')
# plt.step(range(m),my, label='$\dot y$')
# plt.ylabel(r'Velocity $m/s$')
# plt.title('Measurements')
# plt.legend(loc='best',prop={'size':18})


# xt = []
# yt = []
# dxt= []
# dyt= []
# Zx = []
# Zy = []
# Px = []
# Py = []
# Pdx= []
# Pdy= []
# Rdx= []
# Rdy= []
# Kx = []
# Ky = []
# Kdx= []
# Kdy= []

# def savestates(x, Z, P, R, K):
#     xt.append(float(x[0]))
#     yt.append(float(x[1]))
#     dxt.append(float(x[2]))
#     dyt.append(float(x[3]))
#     Zx.append(float(Z[0]))
#     Zy.append(float(Z[1]))
#     Px.append(float(P[0,0]))
#     Py.append(float(P[1,1]))
#     Pdx.append(float(P[2,2]))
#     Pdy.append(float(P[3,3]))
#     Rdx.append(float(R[0,0]))
#     Rdy.append(float(R[1,1]))
#     Kx.append(float(K[0,0]))
#     Ky.append(float(K[1,0]))
#     Kdx.append(float(K[2,0]))
#     Kdy.append(float(K[3,0]))    


# for n in range(len(measurements[0])):

#     # Time Update (Prediction)
#     # ========================
#     # Project the state ahead
#     x = F*x

#     # Project the error covariance ahead
#     P = F*P*F.T + Q

#     # Measurement Update (Correction)
#     # ===============================
#     # Compute the Kalman Gain
#     S = H*P*H.T + R
#     K = (P*H.T) * np.linalg.pinv(S)

#     # Update the estimate via z
#     Z = measurements[:,n].reshape(2,1)
#     y = Z - (H*x)                            # Innovation or Residual
#     x = x + (K*y)

#     # Update the error covariance
#     P = (I - (K*H))*P

#     # Save states (for Plotting)
#     savestates(x, Z, P, R, K)


# def plot_x():
#     fig = plt.figure(figsize=(16,9))
#     plt.step(range(len(measurements[0])),dxt, label='$estimateVx$')
#     plt.step(range(len(measurements[0])),dyt, label='$estimateVy$')

#     plt.step(range(len(measurements[0])),measurements[0], label='$measurementVx$')
#     plt.step(range(len(measurements[0])),measurements[1], label='$measurementVy$')

#     plt.axhline(vx, color='#999999', label='$trueVx$')
#     plt.axhline(vy, color='#999999', label='$trueVy$')

#     plt.xlabel('Filter Step')
#     plt.title('Estimate (Elements from State Vector $x$)')
#     plt.legend(loc='best',prop={'size':11})
#     plt.ylim([0, 30])
#     plt.ylabel('Velocity')
# plot_x()


# def plot_xy():
#     fig = plt.figure(figsize=(16,16))
#     plt.scatter(xt,yt, s=20, label='State', c='k')
#     plt.scatter(xt[0],yt[0], s=100, label='Start', c='g')
#     plt.scatter(xt[-1],yt[-1], s=100, label='Goal', c='r')

#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title('Position')
#     plt.legend(loc='best')
#     plt.axis('equal')
# plot_xy()





