import open3d as o3d
import os
from pc_skeletor import LBC
from pc_skeletor import utility
import numpy as np
from datetime import datetime



def extract_medial_axis(point_cloud, init):
       lbc = LBC(point_cloud=point_cloud,
              init_contraction=init[0],
              init_attraction=init[1],
              max_contraction=2048,
              max_attraction=1024,
              step_wise_contraction_amplification='auto',
              termination_ratio=0.003,

              max_iteration_steps=20,
              down_sample=-1,
              filter_nb_neighbors=20,
              filter_std_ratio=2.0,
              debug=False,
              verbose=False
              )
       lbc.extract_skeleton()
       lbc.extract_topology()

       # variable for export
       global this_time_init
       this_time_init= init
       
       return lbc


#data
path_m = os.path.join(os.getcwd(), "./output_new/2024-05-27_08-12-32-205775_line_2.ply")
path_z = os.path.join(os.getcwd(), "./data/zylinder.ply")
path_colon = os.path.join(os.getcwd(), "./data/Colon.ply")
path_subtriangles2 = os.path.join(os.getcwd(), "./data/Colon_subtriangles_2.ply")

#load 
pcd = o3d.io.read_point_cloud(path_subtriangles2)
mesh = o3d.io.read_triangle_mesh(path_subtriangles2)
pcd_upsampled = mesh.sample_points_uniformly(number_of_points=200000)

init_con_att = np.array(
[[3,0.5], [3,6], [3,2], [3,4], [3,3], [3,1], [1,1]]

)

triangle_mesh = True
if triangle_mesh:
       lbc = extract_medial_axis(mesh, init_con_att[2])
       
       print("first medial axis done")
       pcd = o3d.geometry.PointCloud()
       pcd.points = lbc.pcd.vertices
       lbc.pcd = pcd
else:
       #lbc = extract_medial_axis(pcd, init_con_att[0])       
       lbc = extract_medial_axis(pcd_upsampled, init_con_att[6])
       #lbc = extract_medial_axis(lbc.contracted_point_cloud, init_con_att[0])

print("medial axis done")

#lbc.contracted_point_cloud, _ = lbc.contracted_point_cloud.remove_statistical_outlier(nb_neighbors=20,
#                                                              std_ratio=2.0)

lbc.visualize()
date_time = str(datetime.now())
date_time = date_time.replace(".", "-").replace(":", "-")
date_time = date_time.replace(" ", "_")
lbc.export_results(f'./output/{date_time}___{this_time_init[0]}_{this_time_init[1]}_{triangle_mesh}')
