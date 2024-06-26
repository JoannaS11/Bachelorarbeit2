import json
import get_bspline
import os
import open3d as o3d
import numpy as np


def main():
    current_dir = os.getcwd()
    # json file paths
    path_colon_sub = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__2024-06-17_10-43-34", "colon_subtriangles_06_17_json.json")
    
    json_file_path = path_colon_sub
    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)

        # extract data from json file
        dir_json = os.path.join (*input_liste["dir"])
        data_path = os.path.join(current_dir, *input_liste["data"])
        medial_axis_pcd_path = os.path.join(current_dir, dir_json, *input_liste["medial_axis_pcd"])
        normals_to_inside = input_liste["normals_to_inside"]
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")

        sample_size = 2

        # read point clouds
        pcd_data = o3d.io.read_point_cloud(data_path)
        pcd_medial_axis = o3d.io.read_point_cloud(medial_axis_pcd_path)

        print("after import")

        # create b_spline and export
        line_bSpline = get_bspline.get_bSpline(pcd_medial_axis, sample_size)
        print("after approximation")
        file_name = get_bspline.export_spline_as_json(line_bSpline, f"{sample_size}_bSpline_medial_axis_{data_name}", [dir_json])
        file_dict = {'medial_axis_spline': [file_name]}
        json.dump(file_dict, input_file[0])

if __name__=="__main__": main()