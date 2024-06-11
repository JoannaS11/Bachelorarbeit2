import medial_axis_spheres
import find_minimun_tree
import numpy as np
import os
import open3d as o3d


def find_smaller_pcd(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, distance_point_to_point, normals_inside):
    # adapt normals
    if normals_inside:
        pass
    else:
        zyl_normals = -zyl_normals

    # get big line pointclouds
    pcd_big_line_1, pcd_big_line_2 = medial_axis_spheres.get_big_line_pointcloud(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, distance_point_to_point)

    # export as ply
    medial_axis_spheres.export_pcd_as_ply(pcd_big_line_1, "output_new", f"_{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_pcd_big_path")
    medial_axis_spheres.export_pcd_as_ply(pcd_big_line_2, "output_new", f"_{distance_point_to_point}_{distance_point_to_line}_{mini_residual}_pcd_big_path_without_outlier")

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

def find_min_tree(pcd_big_line_2, pcd_np, max_distance):
    # find line pcd
    mid_line_pcd = find_minimun_tree.find_line(pcd_np, max_distance)

    # export mid line pcd
    find_minimun_tree.export_pcd_as_ply(mid_line_pcd, "output_new", f"{max_distance}_min_path")

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
    path_intestine_short_texture_anim = os.path.join(os.getcwd(), "data","intestine_short_texture_anim.ply")
    path_subtriangles_2 = os.path.join(os.getcwd(), "data","Colon_subtriangles_2.ply")

    #load point clouds
    path = path_z_complex_4
    pcd_colon = o3d.io.read_point_cloud(path_colon)
    pcd = o3d.io.read_point_cloud(path)
    #pcd_1 = o3d.io.read_point_cloud(path_subtriangles_2)

    ################################ medial-axis-spheres ##########################################################
    # get points and normals
    zyl_normals = np.asarray(pcd.normals)
    zyl_points = np.asarray(pcd.points)

    #parameter to change
    mini_residual = np.shape(zyl_points)[0] // 850
    distance_point_to_line = 0.02
    distance_point_to_point = 0.5
    normals_inside = True

    pcd_big_line_2 = find_smaller_pcd(pcd, zyl_points, zyl_normals, mini_residual, distance_point_to_line, distance_point_to_point, normals_inside)

    ########################## find minimum tree ###################################################################

    pcd_np = np.asarray(pcd_big_line_2.points)

    # adjustable parameter
    max_distance = 0.4

    mid_line_pcd = find_min_tree(pcd_big_line_2, pcd_np, max_distance)


if __name__=="__main__": main()