import json
import geomdl.BSpline
import geomdl.exchange
import find_min_distances_to_spline
import os
import open3d as o3d
import numpy as np
import geomdl
from datetime import datetime
import matplotlib.pyplot as plt


def main():
    current_dir = os.getcwd()
    # json file paths
    path_anim_haustren_color_16_10 = os.path.join(current_dir, "output_main", "4_colon_haustren_anim_text2_baked_color__16-10-2024_16-02-50", "4_colon_haustren_anim_text2_baked_color__16-10-2024_16-02-50_json.json")
    path_intestine_14_10_265000 = os.path.join(current_dir, "output_main", "intestine_short_texture_264000__13-10-2024_15-34-22", "intestine_short_texture_264000__13-10-2024_15-34-22_json.json")
    path_int_more_point_11_09_15_44 = os.path.join(current_dir,"output_main", "intestine_short_texture_anim_more_points__11-09-2024_15-44-21", "intestine_short_texture_anim_more_points__11-09-2024_15-44-21_json.json")
    path_sub_14_10 = os.path.join(current_dir, "output_main","Colon_subtriangles_2__14-10-2024_12-31-01", "Colon_subtriangles_2__14-10-2024_12-31-01_json.json")
    path_intestine_14_10_265000 = os.path.join(current_dir, "output_main", "intestine_short_texture_264000__13-10-2024_15-34-22", "intestine_short_texture_264000__13-10-2024_15-34-22_json.json")
    path_intestine_28_10_16577 = os.path.join(current_dir, "output_main", "intestine_short_texture_anim_16577__28-10-2024_14-12-17", "intestine_short_texture_anim_16577__28-10-2024_14-12-17_json.json")
    

    json_file_path = path_intestine_28_10_16577
    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)

        # extract data from json file
        dir_json = os.path.join (*input_liste["dir"])
        data_path = os.path.join(current_dir, *input_liste["data"])
        medial_axis_bspline_path = os.path.join(current_dir, *input_liste["dir"],*input_liste["medial_axis_spline"])
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")
        length_spline = input_liste['length_spline']
        t_on_line_path = input_liste["t_on_line"][1:]

        # read point clouds
        pcd_data = o3d.io.read_point_cloud(data_path)
        medial_axis_bspline = geomdl.exchange.import_json(medial_axis_bspline_path)[0]

        

        # read motion array file
        motion_arrays = np.load(os.path.join(current_dir, dir_json, *t_on_line_path))
        t_on_line = motion_arrays['t_on_line']
        vector_to_line_distances = motion_arrays['vector_to_line_distances']

        # calculate min distances from points to spline
        bin_size = 1 / 200
        local_mins = find_min_distances_to_spline.find_min_distances(vector_to_line_distances, t_on_line, medial_axis_bspline, pcd_data, bin_size, length_spline)
        
        # export as npz
        now = datetime.now()
        date_time = now.strftime("%d-%m-%Y_%H-%M-%S")

        name = f"{date_time}_min_distances.npz"
        np.savez(os.path.join(current_dir, dir_json, *t_on_line_path[:-1], name), local_mins=local_mins)

        # add file name to json file
        input_liste['min_distances_values'][1:]=  [*t_on_line_path[:-1], name]
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)
        print("after adding motion path to json file")


if __name__=="__main__": main()