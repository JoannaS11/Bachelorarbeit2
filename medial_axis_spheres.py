import open3d as o3d
import os
import numpy as np
from datetime import datetime

#data
working_directory = os.getcwd()
print(working_directory)
path_z = os.path.join(working_directory, "data/zylinder_compl-4.ply")
path_colon = os.path.join(os.getcwd(), "data/Colon.ply")
path_subtriangles2 = os.path.join(os.getcwd(), "data/Colon_subtriangles_2.ply")

#load 
triangle_mesh = False
if triangle_mesh:
    pcd = o3d.io.read_triangle_mesh(path_z)
    zylinder_normals = np.asarray(pcd.vertex_normals)
    zylinder_points = np.asarray(pcd.vertices)
else:
    pcd = o3d.io.read_point_cloud(path_subtriangles2)
    zyl_normals = np.asarray(pcd.normals)
    zyl_points = np.asarray(pcd.points)

    pcd.paint_uniform_color([1,0,0])
    #print(np.shape(np.asarray(pcd.colors)))
    colors = np.asarray(pcd.colors)
    colors[:] = [1,1,0]
    mini_residual = 100
    midpoints = np.ndarray([np.shape(zyl_points)[0]//20+1,3])
    for line_start in range(0,np.shape(zyl_points)[0], 20):

        if line_start % 100 == 0:
            print(f"{line_start / np.shape(zyl_points)[0]} done")

        distance = np.ndarray(np.shape(zyl_points)[0])

        tmp = np.cross((zyl_points[:]-zyl_points[line_start]), zyl_normals[line_start])
        #print(np.shape(tmp))
        for x in range(np.shape(zyl_points)[0]):
            distance[x] = np.sqrt(tmp[x][0] **2+ tmp[x][1] **2+ tmp[x][2]**2) / np.sqrt(zyl_normals[line_start][0] **2+ zyl_normals[line_start][1] **2+ zyl_normals[line_start][2]**2)
        #distance[:] = (tmp[:][0] **2+ tmp[:][1] **2+ tmp[:][2]**2) / np.sqrt(zyl_normals[line_start][0] **2+ zyl_normals[line_start][1] **2+ zyl_normals[line_start][2]**2)

        """for x in range(np.shape(zyl_points)[0]):
            tmp = np.cross((zyl_points[x]-zyl_points[line_start]), zyl_normals[x])
            distance[x] = np.sqrt(tmp[0] **2+ tmp[1] **2+ tmp[2]**2) / np.sqrt(zyl_normals[x][0] **2+ zyl_normals[x][1] **2+ zyl_normals[x][2]**2)"""
        distance[line_start] = 100
        #print(distance)
        # get distance indices within threshold from line
        pos_min = np.argwhere(abs(distance) < 0.1)

        
        # calculate distance from each of the points to startpoint        
        points_min = zyl_points[pos_min[:]]
        #print(points_min)
        vector_min = points_min[:]-zyl_points[line_start]
        distance_min = np.ndarray([np.shape(vector_min)[0], 1])
        for i in range(np.shape(distance_min)[0]):
            distance_min[i] = np.sqrt(np.dot(np.reshape(vector_min[i][0], [1,3]) , np.reshape(vector_min[i][0], [3,1])))
        
        # sort distances
        #print(distance_min.shape)
        sort_indices = np.argsort(distance_min, axis = 0)
        pos_min = pos_min[sort_indices[:]]
        #print(pos_min)
        x = 0
        for indexes in pos_min:
            # check that point is not behind the point
            if np.dot(zyl_points[indexes]-zyl_points[line_start], -zyl_normals[line_start]) < 0:
                # check that normals look in contrary directions
                if np.dot(zyl_normals[indexes], zyl_normals[line_start]) < 0:
                    #print(f"indexes {indexes}")
                    #colors[indexes] = [1,1,0]
                    """if distance_min[sort_indices[x]] < 0.2:
                        continue"""
                    midpoints[line_start//20] = zyl_points[line_start] + (zyl_points[indexes] - zyl_points[line_start]) * 0.5
                    break

            x += 1
        #colors[line_start] = [0,1,0]
        #print(f"pos min {pos_min}")
    line = o3d.geometry.PointCloud()
    line.points = o3d.utility.Vector3dVector(midpoints)
    colors_2 = np.asarray(line.colors)
    colors_2[:] = [0,0,0]
    #colors[pos_min] = [1,0,0]
    #colors[5] = [0,1,0]
    #colors[7] = [1,1,0]
    #line.paint_uniform_color([1,0,0])
    #line.colors = o3d.utility.Vector3dVector(colors_2)
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    line.colors = o3d.utility.Vector3dVector(colors_2)
   
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    o3d.io.write_point_cloud(f"./output_new/{date_time}.ply", line)

    line_2,_ = line.remove_statistical_outlier(nb_neighbors=4, std_ratio=2.0)
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    o3d.io.write_point_cloud(f"./output_new/{date_time}_line_2.ply", line_2)
    #print(np.asarray(pcd.colors)[0:10])
    o3d.visualization.draw_geometries([pcd, line],

       mesh_show_wireframe = True,

    mesh_show_back_face = True,
    point_show_normal = True)


