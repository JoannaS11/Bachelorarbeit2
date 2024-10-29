import open3d as o3d
import os
import numpy as np
from datetime import datetime
import math
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from tqdm.autonotebook import tqdm
import copy
import shutil
import json

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
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"), pcd)

def get_big_line_pointcloud(pcd, zyl_points, zyl_normals, mini_residual):
    midpoints = np.full([np.shape(zyl_points)[0]//mini_residual+1, 3], [-999.0, -999.0, -999.0])
    mid_p_dist= np.full([np.shape(zyl_points)[0]//mini_residual+1], [-999.0])
    #midpoints = np.zeros([np.shape(zyl_points)[0]//mini_residual+1,3])

    nbrs = NearestNeighbors(n_neighbors=4, algorithm='ball_tree').fit(zyl_points)

    distances, indices = nbrs.kneighbors(zyl_points)
    mean_distance_point_to_line = np.sum(distances) / (np.shape(distances)[0] * 3)
    #print(f" mean dist poin {mean_distance_point_to_line} and distance shape {np.shape(distances)} abd dist {distances}")

    new_normals = np.zeros([int(np.ceil(np.shape(zyl_normals)[0] / mini_residual)) , 3])
    # iterate over every mini_residual point of pcd
    idx = 0
    for line_start in tqdm(range(0,np.shape(zyl_points)[0], mini_residual), desc="Find big line pcd:"):
        new_normals[idx] = zyl_normals[line_start]
        idx += 1

        # calculate distance from points to line(startpoint + normal)
        distance = np.ndarray(np.shape(zyl_points)[0])
        tmp = np.cross((zyl_points[:]-zyl_points[line_start]), zyl_normals[line_start])
        for x in range(np.shape(zyl_points)[0]):
            distance[x] = np.sqrt(tmp[x][0] **2+ tmp[x][1] **2+ tmp[x][2]**2) / np.sqrt(zyl_normals[line_start][0] **2+ zyl_normals[line_start][1] **2+ zyl_normals[line_start][2]**2)
        
        # get distance indices within threshold from line
        distance[line_start] = 100
        pos_min = np.argwhere(abs(distance) <= mean_distance_point_to_line)

        
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
                    mid_p_dist[line_start//mini_residual] = distance_min[sort_indices[x]]
                    midpoints[line_start//mini_residual] = zyl_points[line_start] + (zyl_points[index] - zyl_points[line_start]) * 0.5
                    break

            x += 1

    # remove not used elements in mid_p_dist array
    mid_p_dist_arg_999 = np.argwhere(mid_p_dist==[-999])
    mid_p_dist = np.delete(mid_p_dist, mid_p_dist_arg_999, axis=0)
    midpoints = np.delete(midpoints, mid_p_dist_arg_999, axis=0)
    new_normals = np.delete(new_normals, mid_p_dist_arg_999, axis=0)
    #plot_mean_and_std(mid_p_dist)
    
    
    # discard too small and too big distances
    mean_dist = np.mean(mid_p_dist)
    std_dist = np.std(mid_p_dist)
    if std_dist > 0:
        mid_p_dist_arg_too_small = np.argwhere(mid_p_dist < [mean_dist - 3 * std_dist])
        mid_p_dist_arg_too_big= np.argwhere(mid_p_dist > [mean_dist + 3 * std_dist])
        midpoints = np.delete(midpoints, np.r_[mid_p_dist_arg_too_small, mid_p_dist_arg_too_big], axis=0)
        new_normals = np.delete(new_normals, np.r_[mid_p_dist_arg_too_small, mid_p_dist_arg_too_big], axis=0)

    # convert to pcd
    line = o3d.geometry.PointCloud()
    line.points = o3d.utility.Vector3dVector(midpoints)
    line.normals = o3d.utility.Vector3dVector(new_normals)

    # second path without outliers
    line_2, _ = line.remove_statistical_outlier(nb_neighbors=4, std_ratio=2.0)

    return line, line_2, mean_dist, mean_distance_point_to_line


def main():
    path = "/home/yn86eniw/2d-gaussian-splatting/output/40c271c0-5/point_cloud/iteration_50000/point_cloud_with_normals.ply"
    object_name = "point_cloud_with_normals.ply"
    current_dir = os.getcwd()
    template_json = os.path.join(current_dir, "output_main", "empty_object_paths_json.json")
    pcd_data = o3d.io.read_point_cloud(path)
    zyl_normals = np.asarray(pcd_data.normals)
    zyl_points = np.asarray(pcd_data.points)
    mini_residual = int(np.max([2, np.shape(zyl_points)[0] // 850]))
    # visualize to see direction of normals
    o3d.visualization.draw_geometries([pcd_data],
        mesh_show_wireframe = True,
        mesh_show_back_face = True,
        point_show_normal = True
    )

    normals_to_inside_answer = input("Are normals pointing to inside? yes: y, no: insert n:  ")
    if normals_to_inside_answer == "y":
        normals_to_inside = True
    elif normals_to_inside_answer == "n":
        normals_to_inside = False
    else:
        raise Exception("wrong input- insert either y or n")
    
    if normals_to_inside:
        pass
    else:
        zyl_normals = -zyl_normals

    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    object_name_without_ply = object_name.replace(".ply","_")
    dir_name_end = f"{object_name_without_ply}_{date_time}"
    dir_name = os.path.join(current_dir, "output_main", dir_name_end)
    os.mkdir(dir_name) 

    ################################## create json file ###########################################################
    json_file_path = os.path.join(dir_name, f"{object_name_without_ply}_{date_time}_json.json")
    open(json_file_path, 'w')
    shutil.copyfile(template_json, json_file_path)

    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)

        # write normals_inside to json file
        input_liste['normals_to_inside'] = normals_to_inside

        # write data_path to json_file
        input_liste["data"] = ["data", object_name]

        # write dir to json_file
        input_liste["dir"] = ["output_main", dir_name_end]

        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

        mini_residual = int(np.max([2, np.shape(zyl_points)[0] // 850]))
        input_liste["mini_residual"] = mini_residual
    
        not_ready = True
        counter = 0
        while not_ready:

            pcd_big_line_with, pcd_big_line_without, mean_distance_point_point, mean_distance_point_to_line = get_big_line_pointcloud(pcd_data, zyl_points, zyl_normals, mini_residual)

            o3d.visualization.draw_geometries([pcd_big_line_with],
                mesh_show_wireframe = True,
                mesh_show_back_face = True,
                point_show_normal = True
            )

            pcd_contracted = input("Is pointcloud enough contracted? yes: y, no: insert n:  ")
            if pcd_contracted == "y":
                not_ready = False
            elif pcd_contracted == "n":
                not_ready = True
            else:
                raise Exception("wrong input- insert either y or n")
            
            zyl_points = np.asarray(pcd_big_line_with.points)
            zyl_normals = np.asarray(pcd_big_line_with.normals)

            export_pcd_as_ply(pcd_big_line_with, dir_name, f"_{mean_distance_point_point}_{mean_distance_point_to_line}_{mini_residual}_pcd_data_big_path_{counter}")
            counter += 1
    


if __name__ == "__main__": main()
