import numpy as np
import os
import open3d as o3d
import scipy
import math
import scipy.interpolate
import geomdl.fitting
from datetime import datetime

def find_distance(start_node, index, max_distance):
    global adj_matr
    global pcd_np
    if np.sum(adj_matr[index]) != 0.000:
        return
    
    for i in range(np.shape(pcd_np)[0]):
        distance = math.dist(start_node,pcd_np[i])
        #print(distance)
        if distance < max_distance and i != index:
            adj_matr[index, i] = distance
            find_distance(pcd_np[i], i, max_distance)
            
# get and prepare Colon cloud for visualization
path_colon = os.path.join(os.getcwd(), "data", "Colon.ply")
pcd_colon = o3d.io.read_point_cloud(path_colon)
pcd_colon.paint_uniform_color([1,1,0])

# get path for mid pointcloud
path_ = os.path.join(os.getcwd(), "output_new", "2024-05-27_08-22-28-505194_line_2.ply")
path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-27_07-48-47-198669_line_2.ply")
path_1 = os.path.join(os.getcwd(), "output_new", "2024-06-03_10-16-16-769887_line_2.ply")
pcd = o3d.io.read_point_cloud(path_1)
pcd_np = (np.asarray(pcd.points))
adj_matr = np.zeros([np.shape(pcd_np)[0], np.shape(pcd_np)[0]])


max_distance = 0.4
find_distance(pcd_np[0], 0, max_distance)
min = np.argmax(adj_matr[0])
"""
dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr,indices = 0, directed=False,unweighted=False, return_predecessors=True)
dist_matrix[dist_matrix==np.inf] = -1
max = np.argmax(dist_matrix)
index = max
path = [max]
while predecessor_array[index] != 0:
    index = predecessor_array[index]
    path.append(index)
"""
dist_matrix, predecessor_array = scipy.sparse.csgraph.dijkstra(adj_matr, directed=False,unweighted=False, return_predecessors=True)
dist_matrix[dist_matrix==np.inf] = -1
max = np.argmax(dist_matrix,axis = 0)
max_2 = np.argmax(dist_matrix[max[:]])
# j = zielpunkt, i = punkt vorher
#print(max[max_2])
#print(dist_matrix[:, max[:]])
print(predecessor_array)
#print(max % np.shape(dist_matrix)[0])
#print(dist_matrix[max // np.shape(dist_matrix)[1], max % np.shape(dist_matrix)[1]])
start_index = max_2
end_index = max[max_2]

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
"""
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
print(np.shape(p))"""

o3d.visualization.draw_geometries([pcd_colon, pcd ,line],

       mesh_show_wireframe = True,

    mesh_show_back_face = True,
    point_show_normal = True)
