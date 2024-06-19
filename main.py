import medial_axis_spheres
import find_minimun_tree
import numpy as np
import os
from datetime import datetime
import open3d as o3d


def find_smaller_pcd(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, distance_point_to_point, normals_inside, dir_name):
    # adapt normals
    if normals_inside:
        pass
    else:
        zyl_normals = -zyl_normals

    # get big line pointclouds
    pcd_big_line_1, pcd_big_line_2 = medial_axis_spheres.get_big_line_pointcloud(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, distance_point_to_point)

    # export as ply
    medial_axis_spheres.export_pcd_as_ply(pcd_big_line_1, "output_new", f"_{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_pcd_big_path", dir_name)
    medial_axis_spheres.export_pcd_as_ply(pcd_big_line_2, "output_new", f"_{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_pcd_big_path_without_outlier", dir_name)

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
    return pcd_big_line_2

def find_min_tree(pcd_big_line_2, pcd_np, max_distance, dir_name):
    # find line pcd
    mid_line_pcd = find_minimun_tree.find_line(pcd_np, max_distance)

    # export mid line pcd
    find_minimun_tree.export_pcd_as_ply(mid_line_pcd, "output_main",f"{max_distance}_min_path", dir_name)

    # color pcd
    mid_line_pcd.paint_uniform_color([1,0,0])
    pcd_big_line_2.paint_uniform_color([0, 1, 0])

    # visualize
    o3d.visualization.draw_geometries([pcd_big_line_2, mid_line_pcd], mesh_show_wireframe = True, mesh_show_back_face = True, point_show_normal = True)

    return mid_line_pcd


def main():
    ################################ data paths ####################################################################
    path_z_complex_4 = os.path.join(os.getcwd(), "data","zylinder_compl-4.ply")
    path_z_complex_2 = os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_simple = os.path.join(os.getcwd(), "data","zylinder_simple.ply")
    path_colon = os.path.join(os.getcwd(), "data","Colon.ply")
    path_colon_seg = os.path.join(os.getcwd(), "data","colon_segments.ply")
    path_intestine_short_texture_anim = os.path.join(os.getcwd(), "data","intestine_short_texture_anim.ply")
    path_subtriangles_2 = os.path.join(os.getcwd(), "data","Colon_subtriangles_2.ply")

    #load point clouds
    path = path_z_complex_4
    pcd_colon = o3d.io.read_point_cloud(path_z_complex_4)
    pcd = o3d.io.read_point_cloud(path)
    #pcd_1 = o3d.io.read_point_cloud(path_subtriangles_2)

    ################################# create directory to save results ###########################################
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    dir_name = f"{path}_{date_time}"
    dir_name = dir_name.replace(".ply","_")
    dir_name = dir_name.replace("data","output_main")
    os.mkdir(dir_name)

    ################################ medial-axis-spheres ##########################################################
    # get points and normals
    zyl_normals = np.asarray(pcd.normals)
    zyl_points = np.asarray(pcd.points)

    #parameter to change
    mini_residual = np.shape(zyl_points)[0] // 850
    distance_point_to_line = 0.07
    min_distance_point_to_point = 0.3
    normals_inside = True

    pcd_big_line_2 = find_smaller_pcd(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, min_distance_point_to_point, normals_inside, dir_name)

    ########################## find minimum tree ###################################################################
    """ can break if pointcloud is not centralized enough!!! ->reaches max recursion depth in comparison"""
    pcd_np = np.asarray(pcd_big_line_2.points)

    # adjustable parameter
    max_distance = 0.4

    mid_line_pcd = find_min_tree(pcd_big_line_2, pcd_np, max_distance, dir_name)

    ######################## centralize minimum tree ############################################################

    ######################### get bSpline #######################################################################


if __name__=="__main__": main()