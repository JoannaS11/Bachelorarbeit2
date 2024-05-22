import open3d as o3d
import os
import numpy as np
from datetime import datetime

#data
working_directory = os.getcwd()
print(working_directory)
path_z = os.path.join(working_directory, "data/zylinder_compl-1.ply")
path_colon = os.path.join(os.getcwd(), "data/Colon.ply")
path_subtriangles2 = os.path.join(os.getcwd(), "data/Colon_subtriangles_2.ply")

#load 
triangle_mesh = False
if triangle_mesh:
    pcd = o3d.io.read_triangle_mesh(path_z)
    zylinder_normals = np.asarray(pcd.vertex_normals)
    zylinder_points = np.asarray(pcd.vertices)
else:
    pcd = o3d.io.read_point_cloud(path_z)
    zylinder_normals = np.asarray(pcd.normals)
    zylinder_points = np.asarray(pcd.points)

    for x in zylinder_points:
        if np.linalg.lstsq(A ,x)[1] < min_residual:
            min_residual = np.linalg.lstsq(A ,x)[1]
            point_huelle = x 




print(zylinder_normals)


