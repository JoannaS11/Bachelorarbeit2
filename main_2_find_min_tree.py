import find_minimun_tree
import numpy as np
import os
from datetime import datetime
import open3d as o3d
import json

def export_pcd_as_ply(pcd, path, output_name_without_ply):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S") 
    name = f"{date_time}_{output_name_without_ply}.ply"   
    o3d.io.write_point_cloud(os.path.join(path, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    return name

def find_min_tree(medial_axis_big_pcd, pcd_data_np, max_distance, path, plot_on=True):
    # find line pcd_data
    mid_line_pcd = find_minimun_tree.find_line(pcd_data_np, max_distance)

    # export mid line pcd_data
    filename = f"{max_distance}_min_path"
    name = export_pcd_as_ply(mid_line_pcd, path,filename)

    if plot_on:
        # color pcd_data
        mid_line_pcd.paint_uniform_color([1,0,0])
        medial_axis_big_pcd.paint_uniform_color([0, 1, 0])

        # visualize
        o3d.visualization.draw_geometries([medial_axis_big_pcd, mid_line_pcd], mesh_show_wireframe = True, mesh_show_back_face = True, point_show_normal = True)

    return mid_line_pcd, name

def main():
    current_dir = os.getcwd()

    path_colon_sub = os.path.join(current_dir,"output_main","Colon_subtriangles_2__02-07-2024_11-22-47","Colon_subtriangles_2_02-07-2024_11-22-47_json.json")
    path_zyl_compl_4 = os.path.join(current_dir,"output_main","zylinder_compl-4__02-07-2024_10-40-05","zylinder_compl-4_02-07-2024_10-40-05_json.json")
    path_zyl_compl_2 = os.path.join(current_dir,"output_main","zylinder_compl-2__02-07-2024_11-03-02","zylinder_compl-2_02-07-2024_11-03-02_json.json")
    path_seg = os.path.join(current_dir,"output_main","colon_segments__02-07-2024_11-06-21","colon_segments_02-07-2024_11-06-21_json.json")
    path_seg_compl = os.path.join(current_dir,"output_main","colon_segments_more_complicated__02-07-2024_11-09-37","colon_segments_more_complicated_02-07-2024_11-09-37_json.json")
    path_in_sh_tex = os.path.join(current_dir,"output_main","intestine_short_texture_anim__02-07-2024_13-24-24","intestine_short_texture_anim__02-07-2024_13-24-24_json.json")
    path_anim_hausten = os.path.join(current_dir,"output_main", "4_colon_haustren_anim_text2__03-07-2024_08-16-41", "4_colon_haustren_anim_text2__03-07-2024_08-16-41_json.json")
    path_seg_compl_8 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__08-07-2024_13-18-56", "colon_segments_more_complicated__08-07-2024_13-18-56_json.json")
    path_anim_haustren_9 = os.path.join(current_dir,"output_main", "4_colon_haustren_anim_text2__09-07-2024_09-50-53", "4_colon_haustren_anim_text2__09-07-2024_09-50-53_json.json" )
    path_sub_09_07 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__09-07-2024_15-17-15", "Colon_subtriangles_2__09-07-2024_15-17-15_json.json")
    path_seg_compl_10_9_39 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__10-07-2024_09-37-50", "colon_segments_more_complicated__10-07-2024_09-37-50_json.json")
    path_sub_10_09_45 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__10-07-2024_09-45-17", "Colon_subtriangles_2__10-07-2024_09-45-17_json.json")

    json_file_path = path_seg_compl_10_9_39

    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)
        """ can break if pointcloud is not centralized enough!!! ->reaches max recursion depth in comparison"""
        medial_axis_big_pcd_path = os.path.join(current_dir, *input_liste["dir"], *input_liste["medial_axis_big_pcd"])
        dir_path = os.path.join(current_dir, *input_liste["dir"])

        medial_axis_big_pcd = o3d.io.read_point_cloud(medial_axis_big_pcd_path)

        pcd_np = np.asarray(medial_axis_big_pcd.points)

        # adjustable parameter
        mean_distance_point_to_point = input_liste["mean_distance_point_to_point"]
        print(mean_distance_point_to_point)

        mid_line_pcd, mid_line_pcd_path = find_min_tree(medial_axis_big_pcd, pcd_np, mean_distance_point_to_point, dir_path)
        input_liste["medial_axis_pcd"] = [mid_line_pcd_path]
        input_liste["max_distance_min_tree"] = mean_distance_point_to_point
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

        print("Find minimum path done")

if __name__=="__main__": main()