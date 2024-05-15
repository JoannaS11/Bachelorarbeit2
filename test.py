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
       
       return lbc

#data
path_z = os.path.join(os.getcwd(), "./data/zylinder.ply")
path_s = os.path.join(os.getcwd(), "./data/Colon.ply")
path_1 = os.path.join(os.getcwd(), "./data/Colon_subtriangles_2.ply")
path_2 = os.path.join(os.getcwd(), "./data/colon-part2.ply")
path_3 = os.path.join(os.getcwd(), "./data/colon-part3.ply")
path_h = "C:/Users/joann/Documents/untitled.ply"

#load 
pcd_ = o3d.io.read_point_cloud(path_1)
mesh = o3d.io.read_triangle_mesh(path_1)
pcd = mesh.sample_points_uniformly(number_of_points=15000)

init_con_att = np.array(
[[3,0.5],[3,6]]

)

o3d.visualization.draw_geometries([pcd_])

# Debug/Visualization
#lbc = extract_medial_axis(mesh, init_con_att[1])
lbc = extract_medial_axis(pcd_, init_con_att[0])
print("first medial axis done")
#lbc_2 = extract_medial_axis(lbc.contracted_point_cloud)

"""pcd = o3d.geometry.PointCloud()
pcd.points = lbc.pcd.vertices
lbc.pcd = pcd"""
lbc.visualize()
lbc.export_results(f'./output/{datetime.now()}')
#lbc_2.visualize()
#lbc_2.export_results('./output/medial_axis')
#path_o = "./output/02_skeleton_LBC.ply"
"""lbc.animate(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]),
            steps=300,
            output='./output')"""

print('visualizing the mesh using open3D')

"""mesh2 = o3d.io.read_point_cloud(path_o)
#print(np.asarray(mesh.vertex_normals))
mesh = o3d.io.read_point_cloud(path_s) 
#print(np.asarray(mesh.vertices))
print("triangles")
#print(np.asarray(mesh.triangles[0:20]))

#normals = o3d.geometry.Normal(mesh)
print(type(mesh.vertices[0]))
medial_axis = np.ndarray(len(mesh.vertices))
for x in range(0,len(mesh.vertices)):
       #print(x)
       mesh.vertices[x] = mesh.vertices[x] + 0.1 * mesh.vertex_normals[x]

       #print(z)


print("there")
print(np.asarray(mesh2.vertices))


o3d.visualization.draw_geometries([mesh2,mesh],

       mesh_show_wireframe = True,

mesh_show_back_face = True)"""
