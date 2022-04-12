import argparse
import os
import numpy as np
import cv2
from tqdm import tqdm
import python.lib.python.NormalEstimatorHough as Estimator

# Main code
save_path = '/home/kl/Pick-and-Place-with-RL/catkin_ws/src/handover/src'
parser = argparse.ArgumentParser(description='Transform depth maps to point cloud given the camera intrinsics')

parser.add_argument('--pc', type=str, default=save_path, help='path to directory saving all the point clouds')
parser.add_argument('--depth_plane', type=str, default=save_path, help='path to directory saving the plane-to-plane depth maps')
parser.add_argument('--depth_point', type=str, default=save_path, help='path to directory saving the point-to-point depth maps')
parser.add_argument('--normal_xyz', type=str, default=save_path, help='path to save all the normals in .xyz file')
parser.add_argument('--normal_img', type=str, default=save_path, help='path to save all the normals in .png file')

opt = parser.parse_args()

# read all the file names
# names = sorted([name.split('.')[0] for name in os.listdir(opt.pc) if name.split('.')[1] == 'xyz'])
name = 'surface_normal'
# for name in tqdm(names, desc='Generating normal maps and point-to-point depth maps'):
normal_xyz_path = os.path.join(opt.normal_xyz, '{}.xyz'.format(name))
normal_img_path = os.path.join(opt.normal_img, '{}.png'.format(name))
pc_path = '/home/kl/Pick-and-Place-with-RL/catkin_ws/src/handover/src/depth2pcl.xyz'
depth_path_plane = os.path.join(opt.depth_plane, '{}.png'.format(name))
depth_path_point = os.path.join(opt.depth_point, '{}.png'.format(name))

# get the H, W of depth map
depth = np.load('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/handover/src/depth_0.npy')[102:122,102:122]
H, W = depth.shape
print(H,W)

# create point-to-point depth map and save in .png of uint-16
pts = np.loadtxt(pc_path)
assert pts.shape[-1] == 3, "The last channel of point cloud must be 3 representing X, Y, Z"
point_depth = np.linalg.norm(pts, 2, -1).reshape(H, W)
assert np.max(point_depth) <= 100, "depth value too large, considering change the normalization ration !"
point_depth_uint = (point_depth * (2 ** 16 - 1) / 100).astype('uint16')
cv2.imwrite(depth_path_point, point_depth_uint)

# create the normal map and save it in .xyz
estimator = Estimator.NormalEstimatorHough()
estimator.set_points(pts)
estimator.set_K(50)
estimator.estimate_normals()
estimator.saveXYZ(normal_xyz_path)

# read in .xyz and transform it to .png of uint-16
# xyz_normal = np.loadtxt(normal_xyz_path)
# normal = xyz_normal[:, 3:].reshape(H, W, 3)
# normal_uint = ((2**16 - 1) * (normal + 1) / 2).astype('uint16')
# cv2.imwrite(normal_img_path, normal_uint)