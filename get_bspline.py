import geomdl.fitting
import numpy as np
import geomdl
import os
import open3d as o3d
import scipy
import matplotlib as plt
import scipy.interpolate
from datetime import datetime

# adjustable parameter
sample_size = 1

# prepare colon for visualization
path_colon = os.path.join(os.getcwd(), "data/Colon.ply")
pcd_colon = o3d.io.read_point_cloud(path_colon)
pcd_colon.paint_uniform_color([1,1,0])

# data paths
path_ = os.path.join(os.getcwd(), "./output_new/2024-05-27_08-22-28-505194_line_2.ply")
path_1 = os.path.join(os.getcwd(), "./output_new/2024-05-29_09-31-42-719176_0.25_min_path.ply")

# read pointcloud and convert to array
pcd = o3d.io.read_point_cloud(path_1)

pcd_np = np.asarray(pcd.points)

# interpolate line and upsample
line_pc_array = np.ndarray.tolist(pcd_np)#line_pc_array)
print("hallo")
x = geomdl.fitting.approximate_curve(line_pc_array, 2)
x = geomdl.fitting.interpolate_curve(line_pc_array, 2)

print(x[0.5])


print(type(x))
print("done")
x.sample_size = sample_size * x.sample_size
y = x.evalpts
x_np = np.array(y)

# create point cloud from curve
line = o3d.geometry.PointCloud()    
line.points = o3d.utility.Vector3dVector(x_np)
line.paint_uniform_color([0,0,1])


# save as ply file
date_time = str(datetime.now())
date_time = date_time.replace(".", "-").replace(":", "-")
date_time = date_time.replace(" ", "_")
o3d.io.write_point_cloud(os.path.join(os.getcwd(), "output_curve", f"{date_time}_curve_as_pointcloud-{sample_size}.ply"), line)

print(f"{np.shape(x_np)}")
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
