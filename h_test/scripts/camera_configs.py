#!/usr/bin/env python
# encoding: utf-8

# filename: camera_configs.py

import cv2
import numpy as np

left_camera_matrix = np.array([[1207.21287547654, 0.000000, 365.415113324103],
			       [0.000000, 1209.57112937799, 246.117113506463],
                               [0.000000, 0.000000, 1.000000]])
left_distortion = np.array([[-0.630381531178977,1.65063406532015,-0.000411398461252554,-0.0173446449792804, 0.000000]])



right_camera_matrix = np.array([[1209.88776209302, 0.000000, 413.856956594846], 
				[0.000000, 1210.67130773907, 228.925227764687], 
				[0.000000, 0.000000, 1.000000]])
right_distortion = np.array([[-0.629241841057911, 0.384306491947133, -0.00302412142395574, -0.0187961427211770, 0.000000]])

#om = np.array([0.01911, 0.03125, -0.00960]) # 旋转关系向量
#R = cv2.Rodrigues(om)[0]  # 使用Rodrigues变换将om变换为R

T = np.array([-502.581451656473,-10.0897997351901,-15.6283011876912]) # 平移关系向量
R = np.array([[0.998453237208844,0.0279672732036317,0.0480516881777589],
	      [-0.0298684171587964,0.998780442220761,0.0393129227319578],
	      [-0.0468936111171582,-0.0406873428335421,0.998070903979040]])
size = (640, 480) # 图像尺寸

# 进行立体更正
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R, T)
# 计算更正map
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

