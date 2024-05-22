
import open3d as o3d
import os
import numpy as np
import scipy.interpolate as scipy_int
import matplotlib.pyplot as plt

path_medial_axis = os.path.join(os.getcwd(), "./output/2024-05-21__17-30-27-523995/02_skeleton_LBC.ply")

pcd = o3d.io.read_point_cloud(path_medial_axis)

#x = scipy_int.UnivariateSpline(np.asarray(pcd.points),np.asarray(pcd.points), s= -71)
#X = np.linspace(min(x), max(x))

#Y = np.linspace(min(y), max(y))

X = np.asarray(pcd.points)[:][0]  # 2D grid for interpolation
Y = np.asarray(pcd.points)[:][1]
Z = np.asarray(pcd.points)[:][2]

interp = scipy_int.LinearNDInterpolator(list(zip(X,Y)), Z)
M = interp(X, Y)
plt.pcolormesh(X, Y, M, shading='auto')

"""o3d.visualization.draw_geometries([interp],

       mesh_show_wireframe = True,

mesh_show_back_face = True)"""