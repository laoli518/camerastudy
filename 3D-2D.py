import numpy as np

# 假设相机内参矩阵K已知
fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5  # 焦距和主点坐标
K = np.array([[fx, 0, cx, 0],
              [0, fy, cy, 0],
              [0, 0, 1, 0]])

# 假设外参矩阵T是一个4x4的齐次变换矩阵
# 这里只是示例值，R是旋转矩阵，t是平移向量
R = np.eye(3)  # 旋转矩阵，如果相机固定不动，没有旋转
t = np.array([0, 0, 0])  # 平移向量，如果相机固定不动，没有平移
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = t

def project_3d_to_2d(points_3d):
    # 将三维点转换为齐次坐标
    points_3d_homogeneous = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))

    # 应用外参变换，将三维点从世界坐标系转换到相机坐标系
    points_3d_camera = T.dot(points_3d_homogeneous.T)

    # 应用内参变换，将相机坐标系的点投影到像素坐标系
    points_2d_homogeneous = K.dot(points_3d_camera)

    # 将齐次坐标转换为二维坐标
    points_2d = points_2d_homogeneous[:2, :] / points_2d_homogeneous[2, :]

    return points_2d.T  # 转置回(N, 2)形状

# 示例：转换一个三维点
point_3d = np.array([1000, 1000, 1000])  # 一个示例三维点
points_2d = project_3d_to_2d(point_3d.reshape(1, -1))

print("3D point:", point_3d)
print("2D point:", points_2d)