import medial_axis_spheres
import find_minimun_tree
import numpy as np
import os
from datetime import datetime
import open3d as o3d
import shutil
import json
import find_bSpline
import find_min_distances_to_spline


def export_pcd_as_ply(pcd, path, output_name_without_ply):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S") 
    name = f"{date_time}_{output_name_without_ply}.ply"   
    o3d.io.write_point_cloud(os.path.join(path, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    return name


def find_smaller_pcd_data(pcd_data, zyl_points, zyl_normals, mini_residual, normals_inside, dir_name):
    # adapt normals
    if normals_inside:
        pass
    else:
        zyl_normals = -zyl_normals

    # get big line pointclouds
    pcd_big_line_with, pcd_big_line_without, mean_distance_point_point, mean_distance_point_to_line = medial_axis_spheres.get_big_line_pointcloud(pcd_data, zyl_points, zyl_normals, mini_residual)

    # export as ply
    filename = f"_{mean_distance_point_point}_{mean_distance_point_to_line}_{mini_residual}_pcd_data_big_path_without_outlier"
    name_with = export_pcd_as_ply(pcd_big_line_with, dir_name, f"_{mean_distance_point_point}_{mean_distance_point_to_line}_{mini_residual}_pcd_data_big_path")
    name_without = export_pcd_as_ply(pcd_big_line_without, dir_name, filename)

    """colors_2 = np.asarray(pcd_big_line_2.colors)
    colors_2[0:100] = [0,0,0]
    colors_2[100:] = [1,0,1]

    pcd_big_line_2.colors = o3d.utility.Vector3dVector(colors_2)
    pcd_data.paint_uniform_color([1,1,0])

    # visualize
    o3d.visualization.draw_geometries([pcd_data, pcd_big_line_2],
        mesh_show_wireframe = True,
        mesh_show_back_face = True,
        point_show_normal = True
    )"""
    return pcd_big_line_with, name_with, pcd_big_line_without, name_without, mean_distance_point_point, mean_distance_point_to_line


def find_min_tree(pcd_big_line_2, pcd_data_np, pcd_data_np_without_outlier, max_distance, dir_name):
    # find line pcd_data
    mid_line_pcd, partial_factor_min_tree = find_minimun_tree.find_line(pcd_data_np, pcd_data_np_without_outlier, max_distance)

    # export mid line pcd_data
    filename = f"{max_distance}_min_path"
    name = export_pcd_as_ply(mid_line_pcd, dir_name,filename)

    # color pcd_data
    mid_line_pcd.paint_uniform_color([1,0,0])
    pcd_big_line_2.paint_uniform_color([0, 1, 0])

    # visualize
    #o3d.visualization.draw_geometries([pcd_big_line_2, mid_line_pcd], mesh_show_wireframe = True, mesh_show_back_face = True, point_show_normal = True)

    return mid_line_pcd, name, partial_factor_min_tree


def main():
    current_dir = os.getcwd()
    template_json = os.path.join(current_dir, "output_main", "empty_object_paths_json.json")
    ################################ data paths ####################################################################
    path_z_complex_4 = "zylinder_compl-4.ply"
    path_z_complex_2 = "zylinder_compl-2.ply"
    path_z_simple = "zylinder_simple.ply"
    path_colon = "Colon.ply"
    path_colon_seg = "colon_segments.ply"
    path_colon_seg_compl = "colon_segments_more_complicated.ply"
    path_intestine_short_texture_anim = "intestine_short_texture_anim.ply"
    path_subtriangles_2 = "Colon_subtriangles_2.ply"
    path_anim_haustren = "4_colon_haustren_anim_text2.ply"

    object_name = path_colon_seg
    path = os.path.join(current_dir, "data", object_name)


    #path = "data_creation_process/colon_part_render0/point_cloud/iteration_30000/point_cloud_with_normals.ply"
    #object_name = "point_cloud_with_normals.ply"

    #load point clouds
    pcd_data = o3d.io.read_point_cloud(path)

    ################################# create directory to save results ###########################################
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

    ################################ medial-axis-spheres ##########################################################
    # get points and normals
    zyl_normals = np.asarray(pcd_data.normals)
    zyl_points = np.asarray(pcd_data.points)

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

        #parameter to change
        mini_residual = int(np.max([2, np.shape(zyl_points)[0] // 850]))
        input_liste["mini_residual"] = mini_residual

        pcd_big_line, pcd_big_line_path,pcd_big_line_without, pcd_big_line_without_path, mean_distance_point_point, max_distance_point_to_line = find_smaller_pcd_data(pcd_data, zyl_points, zyl_normals, mini_residual, normals_to_inside, dir_name)
        input_liste["medial_axis_big_pcd"] = [pcd_big_line_path]
        input_liste["medial_axis_big_pcd_without_outlier"] = [pcd_big_line_without_path]
        input_liste["mean_distance_point_to_point"] = mean_distance_point_point
        input_liste["distance_point_to_line"] = max_distance_point_to_line
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

        print("medial line cloud done")

    ########################## find minimum tree ###################################################################
        """ can break if pointcloud is not centralized enough!!! ->reaches max recursion depth in comparison"""

        # adjustable parameter
        #max_distance = 0.4
        max_distance = input_liste["mean_distance_point_to_point"]
       
        mid_line_pcd, mid_line_pcd_path, partial_factor_min_tree = find_min_tree(pcd_big_line, np.asarray(pcd_big_line.points), np.asarray(pcd_big_line_without.points), max_distance, dir_name)
        input_liste["medial_axis_pcd"] = [mid_line_pcd_path]
        input_liste["max_distance_min_tree"] = partial_factor_min_tree * max_distance
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

        print("Find minimum path done")

    ######################## centralize minimum tree ############################################################

    ######################### get bSpline #######################################################################
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")
        dir_json = os.path.join (*input_liste["dir"])
        sample_size = 2
        input_liste["sample_size_bSpline"] = sample_size

        # create b_spline and export
        medial_axis_bspline = find_bSpline.get_bSpline(mid_line_pcd, sample_size)
        file_name = find_bSpline.export_spline_as_json(medial_axis_bspline, f"{sample_size}_bSpline_medial_axis_{data_name}", [dir_json])

        length_spline = find_bSpline.get_spline_length(medial_axis_bspline)
        # add file name to json file
        input_liste['medial_axis_spline']= [file_name]
        input_liste['length_spline'] = length_spline
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

        print("get b_spline done")

        ############################ distances from points to spline ###########################################
        # get distance of points to mid_line & return all arrays
        vector_to_line, t_on_line, vector_to_line_distances = find_min_distances_to_spline.get_closest_point_on_spline(pcd_data, medial_axis_bspline, normals_to_inside)

        # create dir if it doesn't already exist
        folder_name = "motion_arrays"
        if not os.path.exists(os.path.join(os.getcwd(), dir_json, folder_name)):
            os.mkdir(os.path.join(dir_json, folder_name))
        
        
        # export as npz
        now = datetime.now()
        date_time = now.strftime("%d-%m-%Y_%H-%M-%S")

        name = f"{date_time}_motion_info_arrays.npz"
        np.savez(os.path.join(current_dir, dir_json, folder_name, name), t_on_line=t_on_line, vector_to_line=vector_to_line, vector_to_line_distances=vector_to_line_distances)

        # add file name to json file
        input_liste['t_on_line'][1:]= [folder_name, name]
        input_liste['vector_to_line'][1:]= [folder_name, name]
        input_liste['vector_to_line_distances'][1:]= [folder_name, name]
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

        print("get distances from points to spline done")

        ####################### min distances points to spline #################################################
        # calculate min distances from points to spline
        t_on_line_path = input_liste["t_on_line"][1:]

        bin_size = 1 / 300
        local_mins = find_min_distances_to_spline.find_min_distances(vector_to_line_distances, t_on_line, medial_axis_bspline, pcd_data, bin_size, length_spline, plot_on=False)
        
        # export as npz
        now = datetime.now()
        date_time = now.strftime("%d-%m-%Y_%H-%M-%S")

        name = f"{date_time}_min_distances.npz"
        np.savez(os.path.join(current_dir, dir_json, *t_on_line_path[:-1], name), local_mins=local_mins)

        # add file name to json file
        input_liste['min_distances_values'][1:]=  [*t_on_line_path[:-1], name]
        print("get min distances from points to spline done")

        #############################################################################
        
        # update json_file
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

if __name__=="__main__": main()