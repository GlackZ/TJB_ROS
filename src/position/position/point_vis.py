from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import matplotlib.pyplot as plt

# 从txt文件中读取点云数据
data = np.loadtxt('src/position/position/Mesh.txt')[::50,0:3].T
x, y, z = data[0, :], data[1, :], data[2, :]

# 绕x轴旋转90度的旋转矩阵
theta = np.radians(90)
rotation_matrix = np.array([
    [1, 0, 0],
    [0, np.cos(theta), -np.sin(theta)],
    [0, np.sin(theta), np.cos(theta)]
])

# 应用旋转矩阵到点云数据
rotated_data = rotation_matrix @ data
x_rot, y_rot, z_rot = rotated_data[0, :], rotated_data[1, :], rotated_data[2, :]


# 创建一个新的图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制点云
ax.scatter(x_rot, y_rot, z_rot, c='b', marker='o')

# 设置轴标签
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# 显示图形
plt.show()