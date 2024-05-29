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
path_1 = os.path.join(os.getcwd(), "./output_new/2024-05-29_09-31-42-719176_0.25_min_path.ply")
pcd = o3d.io.read_point_cloud(path_1)
#pcd_np = np.ndarray.tolist(np.asarray(pcd.points))
pcd_np = np.asarray(pcd.points)
#print(pcd_np)

# sort points
line_pc_array = np.ndarray([1, 3])
line_pc_array[0] = pcd_np[0]
#line_pc_array = np.append_along_axis(line_pc_array,pcd_np[0])
print(line_pc_array)
for x in range(np.shape(pcd_np)[0]-1):
    startpoint = pcd_np[x]
    m = (pcd_np[x+1] - pcd_np[x]) / 20
    for x in range(0, 20):
        line_pc_array = np. r_[line_pc_array[:], np.reshape(startpoint + m * x, (1,3))[:]]

print((line_pc_array[:10]))
line_pc_array = np.ndarray.tolist(line_pc_array)
print("hallo")
x = geomdl.fitting.interpolate_curve(line_pc_array, 2)
print(type(x))
print("done")
y = x.evalpts
x_np = np.array(y)
line = o3d.geometry.PointCloud()    
line.points = o3d.utility.Vector3dVector(x_np)
line.paint_uniform_color([0,0,1])
pcd_colon.paint_uniform_color([1,1,0])
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
