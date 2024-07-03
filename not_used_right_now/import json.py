import json
import get_bspline
import os
import open3d as o3d
import numpy as np


def main():
    current_dir = os.getcwd()
    # json file paths
    path_colon_sub = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__2024-06-17_10-43-34", "colon_subtriangles_06_17_json.json")
    path_zyl_compl_4 = os.path.join(current_dir, "output_main", "zylinder_compl-4__2024-06-19_09-04-11", "colon_compl-4_06_19_json.json")
    
    json_file_path = path_zyl_compl_4
    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)

        # extract data from json file
        dir_json = os.path.join (*input_liste["dir"])
        data_path = os.path.join(current_dir, *input_liste["data"])
        normals_to_inside = input_liste["normals_to_inside"]  
        medial_axis_bspline_path = os.path.join(current_dir, *input_liste["dir"],*input_liste["medial_axis_spline"])
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")
        t_on_line_path = input_liste["t_on_line"][1:]

        print(t_on_line_path)

        npzfile = np.load(os.path.join(current_dir, dir_json, *t_on_line_path))
        x = np.linspace(0,1,10)
        npzfile.add(x)

        print(npzfile.files)

        # create b_spline and export

        #vector_to_line, t_on_line, half_line, vector_to_line_distances = get_bspline.get_closest_point_on_spline(pcd_data, medial_axis_bspline, normals_to_inside)

        # create b_spline and export
        """folder_name = "motion_arrays"
        if not os.path.exists(os.path.join(os.getcwd(), dir_json, folder_name)):
            os.mkdir(os.path.join(dir_json, folder_name))
        
        now = datetime.now()
        date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
        # export as ply

        name = f"{date_time}_motion_info_arrays.npz"
        np.savez(os.path.join(current_dir, dir_json, folder_name, name), t_on_line, vector_to_line, vector_to_line_distances)

        # add file name to json file
        input_liste['t_on_line'][2]= [folder_name, name]
        input_liste['vector_to_line'][2]= [folder_name, name]
        input_liste['vector_to_line_distances'][2]= [folder_name, name]
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)
        print("after adding motio path to json file")"""


if __name__=="__main__": main()