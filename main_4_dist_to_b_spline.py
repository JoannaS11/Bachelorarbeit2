import json

import geomdl.BSpline
import geomdl.exchange
import get_bspline
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

    json_file_path = path_anim_hausten
    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)

        # extract data from json file
        dir_json = os.path.join (*input_liste["dir"])
        data_path = os.path.join(current_dir, *input_liste["data"])
        normals_to_inside = input_liste["normals_to_inside"]  
        medial_axis_bspline_path = os.path.join(current_dir, *input_liste["dir"],*input_liste["medial_axis_spline"])
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")
        print(medial_axis_bspline_path)

        # read point clouds
        pcd_data = o3d.io.read_point_cloud(data_path)
        medial_axis_bspline = geomdl.exchange.import_json(medial_axis_bspline_path)[0]

        print("after import")

        # get distance of points to mid_line & return all arrays
        vector_to_line, t_on_line, half_line, vector_to_line_distances = get_bspline.get_closest_point_on_spline(pcd_data, medial_axis_bspline, normals_to_inside)

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
        print("after adding motion path to json file")

if __name__=="__main__": main()