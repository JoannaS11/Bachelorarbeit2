import json
import geomdl.BSpline
import geomdl.exchange
import find_min_distances_to_spline
import os
import open3d as o3d
import numpy as np
import geomdl
from datetime import datetime



def main():
    current_dir = os.getcwd()
    # json file paths
    path_colon_sub = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__02-07-2024_11-22-47", "Colon_subtriangles_2_02-07-2024_11-22-47_json.json")
    path_zyl_compl_4 = os.path.join(current_dir, "output_main", "zylinder_compl-4__02-07-2024_10-40-05", "zylinder_compl-4_02-07-2024_10-40-05_json.json")
    path_zyl_compl_2 = os.path.join(current_dir, "output_main", "zylinder_compl-2__02-07-2024_11-03-02", "zylinder_compl-2_02-07-2024_11-03-02_json.json")
    path_seg = os.path.join(current_dir, "output_main", "colon_segments__02-07-2024_11-06-21", "colon_segments_02-07-2024_11-06-21_json.json")
    path_seg_compl = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__02-07-2024_11-09-37", "colon_segments_more_complicated_02-07-2024_11-09-37_json.json")
    path_anim_hausten = os.path.join(current_dir,"output_main", "4_colon_haustren_anim_text2__03-07-2024_08-16-41", "4_colon_haustren_anim_text2__03-07-2024_08-16-41_json.json")



    json_file_path = path_seg
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
        
        print("after import")

        # calculate min distances from points to spline
        bin_size = length_spline / 500
        local_mins = find_min_distances_to_spline.find_min_distances(vector_to_line_distances, t_on_line, medial_axis_bspline, pcd_data, bin_size)
        
        # export as npz
        now = datetime.now()
        date_time = now.strftime("%d-%m-%Y_%H-%M-%S")

        print(local_mins)
        name = f"{date_time}_min_distances.npz"
        np.savez(os.path.join(current_dir, dir_json, *t_on_line_path[:-1], name), local_mins=local_mins)

        # add file name to json file
        input_liste['min_distances_values'][1:]=  [*t_on_line_path[:-1], name]
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)
        print("after adding motion path to json file")

if __name__=="__main__": main()