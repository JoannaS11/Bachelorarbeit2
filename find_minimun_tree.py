import numpy as np
import os
import open3d as o3d
import scipy
import math
import scipy.interpolate
import geomdl.fitting
from datetime import datetime
from sklearn.neighbors import NearestNeighbors

def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name=None):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    # export as ply
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"), pcd)


def find_distance(start_node, index, max_distance, mean_thickness):
    global adj_matr
    global pcd_np

    if np.sum(adj_matr[index]) != 0:
        return
    
    for i in range(np.shape(pcd_np)[0]):
        distance = math.dist(start_node,pcd_np[i])
        if distance < 0.5 * mean_thickness and i != index:
            adj_matr[index, i] = distance #* (0.3 + distance)
            find_distance(pcd_np[i], i, max_distance, mean_thickness)


def find_line(pcd_np_, mean_thickness):
    #print(pcd_np)
    global pcd_np
    pcd_np = pcd_np_
    global adj_matr
    adj_matr = np.zeros([np.shape(pcd_np)[0], np.shape(pcd_np)[0]])

    nbrs = NearestNeighbors(n_neighbors=10, algorithm='ball_tree').fit(pcd_np)

    distances, indices = nbrs.kneighbors(pcd_np)
    print(distances)
    max_distance_between_points = np.sum(distances) / (np.shape(distances)[0] * (np.shape(distances)[1]-1))
    print(max_distance_between_points)

    find_distance(pcd_np[0], 0, max_distance_between_points, mean_thickness) #max_distance_between_points)

    dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr,indices = 0, directed=False,unweighted=False, return_predecessors=True)
    dist_matrix[dist_matrix==np.inf] = -1
    max = np.argmax(dist_matrix)
    index = max
    path_indexes = [max]
    while predecessor_array[index] != 0:
        index = predecessor_array[index]
        path_indexes.append(index)

    """dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr, directed=False,unweighted=False, return_predecessors=True)
    dist_matrix[dist_matrix==np.inf] = -1
    max = np.argmax(dist_matrix,axis = 0)
    dist_2 = dist_matrix[:, max[:]]
    max_2 = np.argmax(dist_2)#matrix[max[:]])
    # j = zielpunkt, i = punkt vorher
    start_index = max_2
    end_index = dist_matrix[max_2]

    index = end_index
    path_indexes = [end_index]
    while predecessor_array[start_index, index] != start_index:
        index = predecessor_array[start_index, index]
        path_indexes.append(index)"""

    
    mid_line = np.flip(pcd_np[path_indexes[:]], axis=0)

    # convert to pcd
    line = o3d.geometry.PointCloud()
    line.points = o3d.utility.Vector3dVector(mid_line)

    return line
