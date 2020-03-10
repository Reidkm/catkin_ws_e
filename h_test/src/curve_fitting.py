#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np


x = np.arange(500, 1150, 50)
y = np.array([1, 1.111, 1.209, 1.31, 1.429, 1.528, 1.606, 1.692, 1.803, 1.913, 2,2.115,2.2])
#y = np.array([-42, -27, -13, 0, 11, 22, 31, 41, 48, 55, 61])
z1 = np.polyfit(x, y, 1)  # 用3次多项式拟合
p1 = np.poly1d(z1)
print(p1)  # 在屏幕上打印拟合多项式
yvals = p1(x)  # 也可以使用yvals=np.polyval(z1,x)

plot1 = plt.plot(x, y, 'k.', markersize=11, label='$original values$')
plot2 = plt.plot(x, yvals, 'r', lw=3, label='$polyfit values$')
plt.xlabel('depth (mm)')
plt.ylabel(' pixel size (mm/pixel)')
plt.legend(loc=4)  # 指定legend的位置,读者可以自己help它的用法
plt.title('Relationship between depth and pixel size ')
plt.show()



