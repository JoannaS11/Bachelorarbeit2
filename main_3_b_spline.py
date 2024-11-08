import json
import find_bSpline
import os
import open3d as o3d
import numpy as np

def convert_array_to_pcd(np_array, color = [0, 0, 1]):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_array)
    pcd.paint_uniform_color(color)

    return pcd


def plot_bSpline(line_bSpline, pcd_data):
    line_pcd = convert_array_to_pcd(np.asarray(line_bSpline.evalpts), [0,1,0])
    o3d.visualization.draw_geometries(
        [line_pcd, pcd_data],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
    )


def main():
    current_dir = os.getcwd()
    # json file paths
    path_colon_sub = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__02-07-2024_11-22-47", "Colon_subtriangles_2_02-07-2024_11-22-47_json.json")
    path_zyl_compl_4 = os.path.join(current_dir, "output_main", "zylinder_compl-4__02-07-2024_10-40-05", "zylinder_compl-4_02-07-2024_10-40-05_json.json")
    path_zyl_compl_2 = os.path.join(current_dir, "output_main", "zylinder_compl-2__02-07-2024_11-03-02", "zylinder_compl-2_02-07-2024_11-03-02_json.json")
    path_seg = os.path.join(current_dir, "output_main", "colon_segments__02-07-2024_11-06-21", "colon_segments_02-07-2024_11-06-21_json.json")
    path_seg_compl = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__02-07-2024_11-09-37", "colon_segments_more_complicated_02-07-2024_11-09-37_json.json")
    path_anim_hausten = os.path.join(current_dir,"output_main", "4_colon_haustren_anim_text2__03-07-2024_08-16-41", "4_colon_haustren_anim_text2__03-07-2024_08-16-41_json.json")
    path_seg_compl_8 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__08-07-2024_13-18-56", "colon_segments_more_complicated__08-07-2024_13-18-56_json.json")
    path_sub_09_07 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__09-07-2024_15-17-15", "Colon_subtriangles_2__09-07-2024_15-17-15_json.json")
    path_seg_compl_10_9_39 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__10-07-2024_09-37-50", "colon_segments_more_complicated__10-07-2024_09-37-50_json.json")
    path_sub_10_09_45 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__10-07-2024_09-45-17", "Colon_subtriangles_2__10-07-2024_09-45-17_json.json")
    path_seg_compl_29_07_16_01 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__29-07-2024_16-15-42", "colon_segments_more_complicated__29-07-2024_16-15-42_json.json")
    path_anim_hausten_17_9_52 = os.path.join(current_dir, "output_main", "4_colon_haustren_anim_text2__17-07-2024_09-52-26", "4_colon_haustren_anim_text2__17-07-2024_09-52-26_json.json")

    json_file_path = path_sub_10_09_45
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
        line_bSpline = find_bSpline.get_bSpline(pcd_medial_axis, sample_size)
        file_name = find_bSpline.export_spline_as_json(line_bSpline, f"{sample_size}_bSpline_medial_axis_{data_name}", [dir_json])
        plot_bSpline(line_bSpline, pcd_data)

        length_spline = find_bSpline.get_spline_length(line_bSpline)
        # add file name to json file
        input_liste['medial_axis_spline']= [file_name]
        input_liste['length_spline'] = length_spline
        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)
        print("after adding b_spline path to json file")

if __name__=="__main__": main()