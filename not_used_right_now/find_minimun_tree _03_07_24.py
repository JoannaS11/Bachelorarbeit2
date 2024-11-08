import numpy as np
import os
import open3d as o3d
import scipy
import math
import scipy.interpolate
import geomdl.fitting
from datetime import datetime

def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name=None):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    # export as ply
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"), pcd)


def find_distance(start_node, index, max_distance):
    global adj_matr
    global pcd_np

    if np.sum(adj_matr[index]) != 0:
        return
    
    for i in range(np.shape(pcd_np)[0]):
        distance = math.dist(start_node,pcd_np[i])
        #print(distance)
        if distance < max_distance and i != index:
            adj_matr[index, i] = distance
            find_distance(pcd_np[i], i, max_distance)

def find_line(pcd_np_, max_distance):
    #print(pcd_np)
    global pcd_np
    pcd_np = pcd_np_
    global adj_matr
    adj_matr = np.zeros([np.shape(pcd_np)[0], np.shape(pcd_np)[0]])

    find_distance(pcd_np[0], 0, max_distance)

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

def main():
    # get and prepare Colon cloud for visualization
    path_colon = os.path.join(os.getcwd(), "data", "Colon.ply")
    pcd_colon = o3d.io.read_point_cloud(path_colon)
    pcd_colon.paint_uniform_color([1,1,0])

    # paths for mid pointcloud
    path_ = os.path.join(os.getcwd(), "output_new", "2024-05-27_08-22-28-505194_line_2.ply")
    path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-27_07-48-47-198669_line_2.ply")
    path_1 = os.path.join(os.getcwd(), "output_new", "2024-06-03_10-16-16-769887_line_2.ply")
    path_2 = os.path.join(os.getcwd(), "output_new", "2024-06-03_13-38-31-015286__0.2_0.1_1247_line_2.ply")
    path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-07_15-41-24-609048___0.5_0.02_5_pcd_big_path_without_outlier.ply")
    path_zyl_compl = os.path.join(os.getcwd(), "output_new", "2024-06-11_10-22-23-418216__0.5_0.02_9_pcd_big_path_without_outlier.ply")

    # read pcd
    pcd = o3d.io.read_point_cloud(path_zyl_compl)
    pcd_np = np.asarray(pcd.points)

    # adjustable parameter
    max_distance = 0.4

    # find line pcd
    mid_line_pcd = find_line(pcd_np, max_distance)

    # export mid line pcd
    export_pcd_as_ply(mid_line_pcd, "output_new", f"{max_distance}_min_path")

    # color pcd
    mid_line_pcd.paint_uniform_color([1,0,0])
    pcd.paint_uniform_color([0, 1, 0])

    # visualize
    o3d.visualization.draw_geometries([pcd, mid_line_pcd], mesh_show_wireframe = True, mesh_show_back_face = True, point_show_normal = True)
    

if __name__=="__main__": main()

"""# get and prepare Colon cloud for visualization
path_colon = os.path.join(os.getcwd(), "data", "Colon.ply")
pcd_colon = o3d.io.read_point_cloud(path_colon)
pcd_colon.paint_uniform_color([1,1,0])

# get path for mid pointcloud
path_ = os.path.join(os.getcwd(), "output_new", "2024-05-27_08-22-28-505194_line_2.ply")
path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-27_07-48-47-198669_line_2.ply")
path_1 = os.path.join(os.getcwd(), "output_new", "2024-06-03_10-16-16-769887_line_2.ply")
path_2 = os.path.join(os.getcwd(), "output_new", "2024-06-03_13-38-31-015286__0.2_0.1_1247_line_2.ply")
path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-05_13-10-44-833155___0.5_0.02_5_pcd_big_path_without_outlier.ply")
pcd = o3d.io.read_point_cloud(path_zyl_simple)
pcd_np = (np.asarray(pcd.points))
print(np.shape(pcd_np))
adj_matr = np.zeros([np.shape(pcd_np)[0], np.shape(pcd_np)[0]])


max_distance = 0.4
find_distance(pcd_np[0], 0, max_distance)
min = np.argmax(adj_matr[0])

dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr,indices = 0, directed=False,unweighted=False, return_predecessors=True)
dist_matrix[dist_matrix==np.inf] = -1
max = np.argmax(dist_matrix)
index = max
path = [max]
while predecessor_array[index] != 0:
    index = predecessor_array[index]
    path.append(index)

dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr, directed=False,unweighted=False, return_predecessors=True)
dist_matrix[dist_matrix==np.inf] = -1
print(dist_matrix)
max = np.argmax(dist_matrix,axis = 0)
print(max)
print(np.shape(max))
dist_2 = dist_matrix[:, max[:]]
print(f"dist 2 {np.shape(dist_2)}")
max_2 = np.argmax(dist_2)#matrix[max[:]])
print(max_2)
print(np.shape(max_2))
# j = zielpunkt, i = punkt vorher
#print(max[max_2])
#print(dist_matrix[:, max[:]])
print(predecessor_array)
#print(max % np.shape(dist_matrix)[0])
#print(dist_matrix[max // np.shape(dist_matrix)[1], max % np.shape(dist_matrix)[1]])
start_index = max_2
end_index = dist_matrix[max_2]

index = end_index
path = [end_index]
while predecessor_array[start_index, index] != start_index:
    index = predecessor_array[start_index, index]
    path.append(index)

line = o3d.geometry.PointCloud()
mid_line = np.flip(pcd_np[path[:]], axis=0)
#print((mid_line[0:20]))
line.points = o3d.utility.Vector3dVector(mid_line)
line.paint_uniform_color([1,0,0])
colors = np.asarray(line.colors)
#colors[0:10] = [0,0,1]
line.colors = o3d.utility.Vector3dVector(colors)


# export line as point_cloud
date_time = str(datetime.now())
date_time = date_time.replace(".", "-").replace(":", "-")
date_time = date_time.replace(" ", "_")
o3d.io.write_point_cloud(f"./output_new/{date_time}_{max_distance}_min_path.ply", line)

pcd.paint_uniform_color([0, 1, 0])

#print(f" mid {np.shape(mid_line[:][0:2])} {np.shape(mid_line[:][2])}")
#points = scipy.interpolate.griddata(mid_line[:], np.ones(np.shape(mid_line)), mid_line[1:-1])#scipy.interpolate.NearestNDInterpolator(mid_line[:], np.ones(np.shape(mid_line)))
#print(type(points))
#points = scipy.interpolate.NearestNDInterpolator(mid_line[:], np.ones(np.shape(mid_line)))


points = geomdl.fitting.interpolate_curve(mid_line.tolist(), 3)
#p = points#
p = np.asarray(points.points)
#print(np.shape(p))
p1 = o3d.geometry.PointCloud()
#print((mid_line[0:20]))
p1.points = o3d.utility.Vector3dVector(p)
p1.paint_uniform_color([0,0,1])
print(np.shape(p))

o3d.visualization.draw_geometries([pcd_colon, pcd ,line],

       mesh_show_wireframe = True,

    mesh_show_back_face = True,
    point_show_normal = True)
"""