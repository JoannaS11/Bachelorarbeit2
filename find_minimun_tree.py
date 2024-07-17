import numpy as np
import os
import open3d as o3d
import scipy
import math
import scipy.interpolate
import geomdl.fitting
from datetime import datetime
from sklearn.neighbors import NearestNeighbors
from tqdm.autonotebook import tqdm

def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name=None):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    # export as ply
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"), pcd)


def find_distance(start_node, index, mean_thickness, partial_factor, iterIndex, k):
    global adj_matr
    global data

    if np.sum(adj_matr[k][iterIndex, index]) != 0:
        return
    
    for i in range(np.shape(data[k])[0]):
        distance = math.dist(start_node,data[k] [i])
        if distance < partial_factor * mean_thickness and i != index:
            adj_matr[k][iterIndex, index, i] = distance# * (0.3 + distance)
            find_distance(data[k][i], i, mean_thickness, partial_factor, iterIndex, k)


def find_line(pcd_np, pcd_np_without_outlier, mean_thickness):
    partial_factor = [0.375, 0.4, 0.425, 0.45, 0.475, 0.5, 0.525, 0.55, 0.575, 0.6, 0.625, 0.65, 0.675, 0.7, 0.725, 0.75, 0.775, 0.8, 0.825, 0.85] 
    p_number = len(partial_factor)
    path_indexes_collection = {}
    #print(pcd_np)
    global data
    global adj_matr
    adj_matr = {0:np.zeros([p_number,np.shape(pcd_np)[0], np.shape(pcd_np)[0]]),
                1:np.zeros([p_number,np.shape(pcd_np_without_outlier)[0], np.shape(pcd_np_without_outlier)[0]])}

    nbrs = NearestNeighbors(n_neighbors=10, algorithm='ball_tree').fit(pcd_np)

    """    distances, indices = nbrs.kneighbors(pcd_np)
    print(distances)
    max_distance_between_points = np.sum(distances) / (np.shape(distances)[0] * (np.shape(distances)[1]-1))
    print(max_distance_between_points)"""
    data = {0:pcd_np, 
            1:pcd_np_without_outlier}
    path_lengths = np.zeros([2, p_number])
    #print(data)

    for k in range(2):
        for m in tqdm(range(p_number), desc= f"Find min Tree {k}"):
            iterIndex=m
            find_distance(data[k][0], 0, mean_thickness, partial_factor[m], iterIndex, k) #max_distance_between_points)

            """dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr,indices = 0, directed=False,unweighted=False, return_predecessors=True)
            dist_matrix[dist_matrix==np.inf] = -1
            max = np.argmax(dist_matrix)
            index = max
            path_indexes = [max]
            while predecessor_array[index] != 0:
                index = predecessor_array[index]
                path_indexes.append(index)"""

            dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr[k][m], directed=False,unweighted=False, return_predecessors=True)
            dist_matrix[dist_matrix==np.inf] = -1
            max = np.argmax(dist_matrix,axis = 0)
            dist_2 = np.zeros(np.shape(dist_matrix)[0])
            for i in range(np.shape(dist_matrix)[0]):
                dist_2[i] = dist_matrix[i, max[i]]
            #print(f" dist matrix {np.shape(dist_matrix)} and mand max shape{np.shape(max)} and {max} dist 2 {np.shape(dist_2)}")
            max_2 = np.argmax(dist_2)#matrix[max[:]])
            #print(f" dist matrix {np.shape(dist_matrix)} and max_2 {max_2}   and max shape{np.shape(max)} and {max} dist 2 {np.shape(dist_2) } adn dist2 {dist_2}")
            # j = zielpunkt, i = punkt vorher
            start_index = max_2
            end_index = max[max_2]

            index = end_index
            path_indexes = [end_index]
            while predecessor_array[start_index, index] != start_index:
                index = predecessor_array[start_index, index]
                path_indexes.append(index)
            #print(np.shape(path_indexes))
            path_indexes_collection[f'{k}{m}'] = path_indexes

            length = 0
            for x in range(len(path_indexes_collection[f'{k}{m}'])-1):
                length += math.dist(data[k][path_indexes_collection[f'{k}{m}'][x]], data[k][path_indexes_collection[f'{k}{m}'][x+1]])
            path_lengths[k, m] = length
    

    print(path_lengths)
    longest_path = np.argwhere(path_lengths == np.max(path_lengths))[0]
    print(longest_path)
    mid_line = np.flip(data[longest_path[0]][path_indexes_collection[f'{longest_path[0]}{longest_path[1]}']], axis=0)

    # convert to pcd
    line = o3d.geometry.PointCloud()
    line.points = o3d.utility.Vector3dVector(mid_line)

    return line, partial_factor[longest_path[1]]
