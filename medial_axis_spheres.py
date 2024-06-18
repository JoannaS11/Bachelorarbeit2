import open3d as o3d
import os
import numpy as np
from datetime import datetime
import math
import matplotlib.pyplot as plt

def plot_mean_and_std(values):
    mean_values = np.mean(values)
    std_values = np.std(values)
    
    print(mean_values)
    print(std_values)
    
    plt.hist(values, width=0.01)
    plt.axvline(mean_values, color = 'g', linestyle = '--', linewidth = 3)
    plt.axvline(mean_values + std_values, color = 'r',linestyle = '--', linewidth = 3)
    plt.axvline(mean_values - std_values, color = 'r',linestyle = '--', linewidth = 3)
    plt.show()

def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name = None):
    # get current date and time
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"), pcd)


def get_big_line_pointcloud(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, distance_point_to_point):
    midpoints = np.full([np.shape(zyl_points)[0]//mini_residual+1, 3], [-999.0, -999.0, -999.0])
    mid_p_dist= np.full([np.shape(zyl_points)[0]//mini_residual+1], [-999.0])
    #midpoints = np.zeros([np.shape(zyl_points)[0]//mini_residual+1,3])

    # iterate over every mini_residual point of pcd
    for line_start in range(0,np.shape(zyl_points)[0], mini_residual):
        
        # 
        if line_start % (mini_residual * 5) == 0:
            print(f"{np.round(line_start / np.shape(zyl_points)[0], decimals=3) * 100} % done")

        # calculate distance from points to line(startpoint + normal)
        distance = np.ndarray(np.shape(zyl_points)[0])
        tmp = np.cross((zyl_points[:]-zyl_points[line_start]), zyl_normals[line_start])
        for x in range(np.shape(zyl_points)[0]):
            distance[x] = np.sqrt(tmp[x][0] **2+ tmp[x][1] **2+ tmp[x][2]**2) / np.sqrt(zyl_normals[line_start][0] **2+ zyl_normals[line_start][1] **2+ zyl_normals[line_start][2]**2)
        
        # get distance indices within threshold from line
        distance[line_start] = 100
        pos_min = np.argwhere(abs(distance) < distance_point_to_line)

        
        # calculate distance from each of the points to startpoint        
        points_min = zyl_points[pos_min[:]]
        vector_min = points_min[:]-zyl_points[line_start]
        distance_min = np.ndarray([np.shape(points_min)[0], 1])
        for i in range(np.shape(distance_min)[0]):
            distance_min[i] = np.sqrt(np.dot(np.reshape(vector_min[i][0], [1,3]) , np.reshape(vector_min[i][0], [3,1])))#math.dist(points_min[i], np.reshape(x))
        
        # sort distances
        sort_indices = np.argsort(distance_min, axis = 0)
        pos_min = pos_min[sort_indices[:]]

        x = 0
        for index in pos_min:
            # check that point is not behind the point
            if np.dot(zyl_points[index]-zyl_points[line_start], - (zyl_normals[line_start])) < 0:
                    # check that normals look in contrary directions
                if np.dot(zyl_normals[index], zyl_normals[line_start]) < 0:
                    if distance_min[sort_indices[x]] < distance_point_to_point:
                        break
                    mid_p_dist[line_start//mini_residual] = distance_min[sort_indices[x]]
                    midpoints[line_start//mini_residual] = zyl_points[line_start] + (zyl_points[index] - zyl_points[line_start]) * 0.5
                    break

            x += 1

    # remove not used elements in mid_p_dist array
    mid_p_dist_arg_999 = np.argwhere(mid_p_dist==[-999])
    mid_p_dist = np.delete(mid_p_dist, mid_p_dist_arg_999, axis=0)

    """plot_mean_and_std(mid_p_dist)
    
    # discard too small and too big distances
    mean_dist = np.mean(mid_p_dist)
    std_dist = np.std(mid_p_dist)
    if std_dist > 0:
        mid_p_dist_arg_too_small = np.argwhere(mid_p_dist==[mean_dist - std_dist])
        midpoints = np.delete(midpoints, mid_p_dist_arg_too_small, axis=0)
        mid_p_dist_arg_too_big= np.argwhere(mid_p_dist==[mean_dist + std_dist])
        midpoints = np.delete(midpoints, mid_p_dist_arg_too_big, axis=0)"""

    # remove not used elements in midpoints array
    midpoints_arg_999 = np.argwhere(midpoints==[-999, -999, -999])
    midpoints = np.delete(midpoints, midpoints_arg_999[::3,0], axis=0)


    # convert to pcd
    line = o3d.geometry.PointCloud()
    line.points = o3d.utility.Vector3dVector(midpoints)

    # second path without outliers
    line_2, _ = line.remove_statistical_outlier(nb_neighbors=4, std_ratio=2.0)

    return line, line_2

def main():
    #data paths
    path_z_complex = os.path.join(os.getcwd(), "data","zylinder_compl-4.ply")
    path_z_simple = os.path.join(os.getcwd(), "data","zylinder_simple.ply")
    path_colon = os.path.join(os.getcwd(), "data","Colon.ply")
    path_intestine_short_texture_anim = os.path.join(os.getcwd(), "data","intestine_short_texture_anim.ply")
    path_subtriangles_2 = os.path.join(os.getcwd(), "data","Colon_subtriangles_2.ply")

    #load point clouds
    path = path_z_complex
    pcd_colon = o3d.io.read_point_cloud(path_colon)
    pcd = o3d.io.read_point_cloud(path)
    #pcd_1 = o3d.io.read_point_cloud(path_subtriangles_2)

    # get points and normals
    zyl_normals = np.asarray(pcd.normals)
    zyl_points = np.asarray(pcd.points)
    print(np.shape(zyl_points))

    #parameter to change
    mini_residual = np.shape(zyl_points)[0] // 850
    distance_point_to_line = 0.02
    distance_point_to_point = 0.5
    normals_inside = True

    # adapt normals
    if normals_inside:
        pass
    else:
        zyl_normals = -zyl_normals

    # get big line pointclouds
    pcd_big_line_1, pcd_big_line_2 = get_big_line_pointcloud(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, distance_point_to_point)

    # export as ply
    export_pcd_as_ply(pcd_big_line_1, "output_new", f"_{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_pcd_big_path")
    export_pcd_as_ply(pcd_big_line_2, "output_new", f"_{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_pcd_big_path_without_outlier")

    colors_2 = np.asarray(pcd_big_line_2.colors)
    colors_2[0:100] = [0,0,0]
    colors_2[100:] = [1,0,1]

    pcd_big_line_2.colors = o3d.utility.Vector3dVector(colors_2)
    pcd.paint_uniform_color([1,1,0])

    # visualize
    o3d.visualization.draw_geometries([pcd, pcd_big_line_2],
        mesh_show_wireframe = True,
        mesh_show_back_face = True,
        point_show_normal = True
    )

if __name__ == "__main__": main()