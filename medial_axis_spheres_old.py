import open3d as o3d
import os
import numpy as np
from datetime import datetime
import math

#data paths
path_z = os.path.join(os.getcwd(), "data","zylinder_compl-4.ply")
path_colon = os.path.join(os.getcwd(), "data","Colon.ply")
path_colon_2 = os.path.join(os.getcwd(), "data","intestine_short_texture_anim.ply")
path_subtriangles2 = os.path.join(os.getcwd(), "data","Colon_subtriangles_2.ply")


#load point clouds
pcd_colon = o3d.io.read_point_cloud(path_colon)
pcd = o3d.io.read_point_cloud(path_colon_2)
#pcd = o3d.io.read_point_cloud(path_subtriangles2)

# get points and normals
zyl_normals = np.asarray(pcd.normals)
zyl_points = np.asarray(pcd.points)

#parameter to change
mini_residual = np.shape(zyl_points)[0] // 850
distance_point_to_line = 0.1
distance_point_to_point = 0.2
normals_inside = False

# adapt normals
if normals_inside:
    pass
else:
    zyl_normals = - zyl_normals

print(np.shape(zyl_points))


pcd.paint_uniform_color([1,0,0])
colors = np.asarray(pcd.colors)
colors[:] = [1,1,0]


#midpoints = np.full([np.shape(zyl_points)[0]//mini_residual+1,3], [-999, -999, -999])
midpoints = np.zeros([np.shape(zyl_points)[0]//mini_residual+1,3])
#midpoints = np.zeros([np.shape(zyl_points)[0]//mini_residual+1,3])
for line_start in range(0,np.shape(zyl_points)[0], mini_residual):

    if line_start % (mini_residual * 5) == 0:
        print(f"{line_start / np.shape(zyl_points)[0]} done")

    distance = np.ndarray(np.shape(zyl_points)[0])

    tmp = np.cross((zyl_points[:]-zyl_points[line_start]), zyl_normals[line_start])
    for x in range(np.shape(zyl_points)[0]):
        distance[x] = np.sqrt(tmp[x][0] **2+ tmp[x][1] **2+ tmp[x][2]**2) / np.sqrt(zyl_normals[line_start][0] **2+ zyl_normals[line_start][1] **2+ zyl_normals[line_start][2]**2)
    #distance[:] = (tmp[:][0] **2+ tmp[:][1] **2+ tmp[:][2]**2) / np.sqrt(zyl_normals[line_start][0] **2+ zyl_normals[line_start][1] **2+ zyl_normals[line_start][2]**2)

    distance[line_start] = 100
    #print(distance)
    # get distance indices within threshold from line
    pos_min = np.argwhere(abs(distance) < distance_point_to_line)

    
    # calculate distance from each of the points to startpoint        
    points_min = zyl_points[pos_min[:]]
    #print(points_min)
    vector_min = points_min[:]-zyl_points[line_start]
    distance_min = np.ndarray([np.shape(points_min)[0], 1])
    for i in range(np.shape(distance_min)[0]):
        distance_min[i] = np.sqrt(np.dot(np.reshape(vector_min[i][0], [1,3]) , np.reshape(vector_min[i][0], [3,1])))#math.dist(points_min[i], np.reshape(x))
    
    # sort distances
    #print(distance_min.shape)
    sort_indices = np.argsort(distance_min, axis = 0)
    pos_min = pos_min[sort_indices[:]]
    #print(pos_min)
    x = 0
    for index in pos_min:
        # check that point is not behind the point
        if np.dot(zyl_points[index]-zyl_points[line_start], - (zyl_normals[line_start])) < 0:
                # check that normals look in contrary directions
            if np.dot(zyl_normals[index], zyl_normals[line_start]) < 0:
                #print(f"indexes {indexes}")
                #colors[indexes] = [1,1,0]
                if distance_min[sort_indices[x]] < distance_point_to_point:
                    break
                midpoints[line_start//mini_residual] = zyl_points[line_start] + (zyl_points[index] - zyl_points[line_start]) * 0.5
                break

        x += 1
    #colors[line_start] = [0,1,0]
    #print(f"pos min {pos_min}")
"""midpoints[1] = [1,1,1]
empty_points_x,empty_points_y = np.where(midpoints == [-999, -999, -999])
print([empty_points_x[:], empty_points_y[:]])
print("here")
print(midpoints)
print("thter")
p = np.c_[empty_points_x, empty_points_y]
print(p)
midpoints = np.delete(midpoints, p[:])
print(midpoints)"""
line = o3d.geometry.PointCloud()
line.points = o3d.utility.Vector3dVector(midpoints)
colors_2 = np.asarray(line.colors)
colors_2[0:100] = [0,0,0]
colors_2[100:] = [1,0,1]

#colors[pos_min] = [1,0,0]
#colors[5] = [0,1,0]
#colors[7] = [1,1,0]
#line.paint_uniform_color([1,0,0])
#line.colors = o3d.utility.Vector3dVector(colors_2)

line.colors = o3d.utility.Vector3dVector(colors_2)
pcd.colors = o3d.utility.Vector3dVector(colors)


date_time = str(datetime.now())
date_time = date_time.replace(".", "-").replace(":", "-")
date_time = date_time.replace(" ", "_")
o3d.io.write_point_cloud(os.path.join(os.getcwd(), "output_new", f"{date_time}__{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_line.ply"), line)

line_2,_ = line.remove_statistical_outlier(nb_neighbors=4, std_ratio=2.0)
date_time = str(datetime.now())
date_time = date_time.replace(".", "-").replace(":", "-")
date_time = date_time.replace(" ", "_")
o3d.io.write_point_cloud(os.path.join(os.getcwd(), "output_new", f"{date_time}__{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_line_2.ply"), line_2)
print(np.shape(midpoints))
#print(np.asarray(pcd.colors)[0:10])
o3d.visualization.draw_geometries([pcd, line_2],

    mesh_show_wireframe = True,

mesh_show_back_face = True,
point_show_normal = True)


