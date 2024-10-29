import open3d as o3d
import numpy as np
import os

def convert_array_to_pcd(np_array, color):
    pcd = o3d.geometry.PointCloud()
    np_array = np.asarray(np_array)
    pcd.points = o3d.utility.Vector3dVector(np_array)
    pcd.paint_uniform_color(color)

    return pcd

def plot_midline_as_pcd(pcd_data, laplace_pcd):
    #midline_pcd = convert_array_to_pcd(np.asarray(laplace_pcd), [0, 0, 0])

    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultLitTransparency"
    mat.base_color = [0.0, 0.0, 1.0, 0.4]

    #midline_pcd.paint_uniform_color([0,0,0])

    mat2 = o3d.visualization.rendering.MaterialRecord()
    mat2.base_color = [0.0, 0.0, 0.0, 0.2]
    mat2.point_size=5


    o3d.visualization.draw([{'name':'colon', 'geometry':pcd_data, "material": mat},{'name': 'mid_line', 'geometry': laplace_pcd, 'material':mat2}])


data_path = os.path.join(os.getcwd(), "data", "Colon_subtriangles_2.ply")
laplace_path = "output/2024-05-22_09-16-47-252112___3.0_3.0_False/02_skeleton_LBC.ply"

pcd_data = o3d.io.read_point_cloud(data_path)
laplace_data = o3d.io.read_point_cloud(laplace_path)
plot_midline_as_pcd(pcd_data, laplace_data)