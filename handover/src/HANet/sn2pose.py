import os
import numpy as np
from math import atan, tan, pi
from scipy.spatial.transform import Rotation
from math import *
import matplotlib.pyplot as plt
import python.lib.python.NormalEstimatorHough as Estimator

class SurfaceNormal2Quaternion():
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.wild = 20

    def get_quaternion(self, depth, pos_x, pos_y, theta, ratio=1e-3):
        H, W = depth.shape
        self.fov_x = 2 * atan(W / (2 * self.fx))
        self.fov_y = 2 * atan(H / (2 * self.fy))
        point_cloud = []

        # change value to meters
        img = depth.astype('float64')
        img *= ratio

        for i in range(pos_x - self.wild, pos_x + self.wild + 1, 1):
            for j in range(pos_y - self.wild, pos_y + self.wild + 1, 1):
                alpha = (pi - self.fov_x) / 2
                gamma = alpha + self.fov_x * float((W - j) / W)
                delta_x = img[i, j] / tan(gamma)

                alpha = (pi - self.fov_y) / 2
                gamma = alpha + self.fov_y * float((H - i) / H)
                delta_y = img[i, j] / tan(gamma)

                point_cloud.append([delta_x, delta_y, float(img[i, j])])

        estimator = Estimator.NormalEstimatorHough()
        estimator.set_points(np.array(point_cloud))
        estimator.set_K(50)
        estimator.estimate_normals()
        tmp = estimator.get_normals()[int((2*self.wild + 1)/2)]
        tmp2 = -1*np.array(tmp)
        rotations_vector = np.array([tmp2[2], tmp2[1], tmp2[0]])
        print(rotations_vector)
        r = Rotation.from_rotvec(theta * rotations_vector, degrees=True)
        q = r.as_quat()

        return q



a = 387.59381103515625
b = 387.59381103515625
c = 321.61895751953125
d = 239.87408447265625


# Input : depth image, instere (x, y), theta / Output : Quaternion
# depth_data_path = '/home/kl/Pick-and-Place-with-RL/catkin_ws/src/handover/src/depth_0.npy'
# depth_img = np.load(depth_data_path)
depth_img = np.zeros((224,224))
depth_img += 15000
depth_img[50:200, 50:200] = 2000
# plt.imshow(depth_img)
# plt.show()


transformer = SurfaceNormal2Quaternion(a,b,c,d)

Q = transformer.get_quaternion(depth_img, 112, 112, 90)
print(Q)