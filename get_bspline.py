import geomdl.fitting
import numpy as np
import geomdl
import os
import open3d as o3d
import scipy
import matplotlib as plt
import scipy.interpolate


path_colon = os.path.join(os.getcwd(), "data/Colon.ply")
pcd_colon = o3d.io.read_point_cloud(path_colon)

path_ = os.path.join(os.getcwd(), "./output_new/2024-05-27_08-22-28-505194_line_2.ply")
path_1 = os.path.join(os.getcwd(), "./output_new/2024-05-27_07-48-47-198669_line_2.ply")
pcd = o3d.io.read_point_cloud(path_1)
pcd_np = np.ndarray.tolist(np.asarray(pcd.points)[0:300])
#print(pcd_np)

# sort points




print("hallo")
x = geomdl.fitting.interpolate_curve(pcd_np, 2)
print(type(x))
print("done")
y = x.evalpts
x_np = np.array(y)
line = o3d.geometry.PointCloud()    
line.points = o3d.utility.Vector3dVector(x_np)
print("start")
o3d.visualization.draw_geometries([line, pcd_colon],

       mesh_show_wireframe = True,

    mesh_show_back_face = True,
    point_show_normal = True)
print("done")
"""
bspl = scipy.interpolate.make_interp_spline(pcd_np[:][0], pcd_np[:][1:3], k=5, axis=1)
xx = np.linspace(0, 2*np.pi, 100)
ax = plt.axes(projection='3d')
ax.plot3D(xx, *bspl(xx))
#ax.scatter3D(x, *y, color='red')
plt.show()"""
