import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from sklearn.neighbors import KDTree
# import ros2pkg
# import ros2pkg.resource
# sys.path.append(f'/home/anaconda3/envs/isaac/lib/python3.10/site-packages/scikit-learn/') 

def IterativeClosestPoint(source_pts, target_pts, tau=10e-6):
    '''
    This function implements iterative closest point algorithm based 
    on Besl, P.J. & McKay, N.D. 1992, 'A Method for Registration 
    of 3-D Shapes', IEEE Transactions on Pattern Analysis and Machine 
    Intelligence, vol. 14, no. 2,  IEEE Computer Society. 

    inputs:
    source_pts : 3 x N
    target_pts : 3 x M
    tau : threshold for convergence
    Its the threshold when RMSE does not change comapred to the previous 
    RMSE the iterations terminate. 

    outputs:
    R : Rotation Matrtix (3 x 3)
    t : translation vector (3 x 1)
    k : num_iterations
    '''

    k = 0
    current_pts = source_pts.copy()
    last_rmse = 0
    t = np.zeros((3, 1))
    R = np.eye(3, 3)

    # iteration loop
    while True:
        neigh_pts = FindNeighborPoints(current_pts, target_pts)
        (R, t) = RegisterPoints(source_pts, neigh_pts)
        current_pts = ApplyTransformation(source_pts, R, t)
        rmse = ComputeRMSE(current_pts, neigh_pts)
        # print("iteration : {}, rmse : {}".format(k,rmse))

        if np.abs(rmse - last_rmse) < tau:
            break
        last_rmse = rmse
        k = k + 1

    return (R, t, k)


# Computes the root mean square error between two data sets.
# here we dont take mean, instead sum.
def ComputeRMSE(p1, p2):
    return np.sum(np.sqrt(np.sum((p1-p2)**2, axis=0)))


# applies the transformation R,t on pts
def ApplyTransformation(pts, R, t):
    return np.dot(R, pts) + t

# applies the inverse transformation of R,t on pts
def ApplyInvTransformation(pts, R, t):
    return np.dot(R.T,  pts - t)

# calculate naive transformation errors
def CalcTransErrors(R1, t1, R2, t2):
    Re = np.sum(np.abs(R1-R2))
    te = np.sum(np.abs(t1-t2))
    return (Re, te)


# point cloud registration between points p1 and p2
# with 1-1 correspondance
def RegisterPoints(p1, p2):
    u1 = np.mean(p1, axis=1).reshape((3, 1))
    u2 = np.mean(p2, axis=1).reshape((3, 1))
    pp1 = p1 - u1
    pp2 = p2 - u2
    W = np.dot(pp1, pp2.T)
    U, _, Vh = np.linalg.svd(W)
    R = np.dot(U, Vh).T
    if np.linalg.det(R) < 0:
        Vh[2, :] *= -1
        R = np.dot(U, Vh).T
    t = u2 - np.dot(R, u1)
    return (R, t)


# function to find source points neighbors in
# target based on KDTree
def FindNeighborPoints(source, target):
    n = source.shape[1]
    kdt = KDTree(target.T, leaf_size=30, metric='euclidean')
    index = kdt.query(source.T, k=1, return_distance=False).reshape((n,))
    return target[:, index]

# 将点云数据转换为icp算法所需的格式
def point_trans(points):
    icp_points = np.array(points).T
    return icp_points

# 读取实际点云数据
def read_ply(file_path):
    target_points = np.loadtxt(file_path).T
    return target_points

# 计算相机光心到机器人坐标系的变换矩阵
def camera_to_robot():
    # 记得回来修改
    t = np.array([0, 0, 0]).reshape(3, 1)
    theta = 0
    phi = 0
    psi = 0
    R = Rot.from_euler('zyx', [theta, phi, psi], degrees = True)
    R = R.as_matrix()
    return R, t

# 获取icp算法计算的位置
def get_position(points):
    source_pts = point_trans(points)
    # target_pts = read_ply('src/position/position/bunny.txt')
    target_pts = np.loadtxt('src/position/position/Mesh.txt')[::50,0:3].T
    # start = time.time()
    Rr, tr, num_iter = IterativeClosestPoint(source_pts, target_pts, tau = 10e-6)
    # end = time.time()
    R_ctr, t_ctr = camera_to_robot()
    R = np.dot(Rr, R_ctr)
    t = np.dot(Rr, t_ctr) + tr
    euler_angles = Rot.from_matrix(R).as_euler('zyx', degrees = True)
    theta = format(euler_angles[0], '.4f')
    phi = format(euler_angles[1], '.4f')
    psi = format(euler_angles[2], '.4f')
    x = format(t[0, 0], '.4f')
    y = format(t[1, 0], '.4f')
    z = format(t[2, 0], '.4f')
    position = f"x:{x}, y:{y}, z:{z}, theta:{theta}, phi:{phi}, psi:{psi}"
    return position

if __name__ == '__main__':
    print(sys.path)
    points = [(1, 2, 3), (4, 5, 6), (7, 8, 9)]
    print(get_position(points))