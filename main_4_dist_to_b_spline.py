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


def plot_vectors(pcd_colon, vector_to_line):
    fig = plt.figure()

    print(f" t_on {np.shape(pcd_colon)} and vector_to {np.shape(vector_to_line)}")
    ax = fig.add_subplot()
    ax.scatter(pcd_colon, vector_to_line)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    #ax.set_zlabel('z')

    plt.show()

def plot_3d(vector_to_line, pcd_data, mid_line):

    fig = plt.figure()
    print("hallo")

    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(mid_line[:,0], mid_line[:,1], mid_line[:,2], color = 'c')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

    fig = plt.figure()
    print("hallo")

    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=0, azim=90, roll=0)
    ax.quiver(pcd_data[:20,0], pcd_data[:20,1], pcd_data[:20,2], vector_to_line[:20,0], vector_to_line[:20,1], vector_to_line[:20,2], color='g')
    #ax.quiver(pcd_colon[1000:,0], pcd_colon[1000:,1], pcd_colon[1000:,2], vector_to_line[1000:,0], vector_to_line[1000:,1], vector_to_line[1000:,2], color='b')
    #ax.plot(start_points[:,0], start_points[:,1], start_points[:,2], color = 'r')
    ax.scatter(mid_line[:,0], mid_line[:,1], mid_line[:,2], color = 'c')
    #ax.scatter(pcd_colon[759,0],pcd_colon[759,1], pcd_colon[759,2], color = 'b')
    #ax.scatter(l[0],l[1], l[2], color = 'b')
    # Set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

def convert_array_to_pcd(np_array, color = [0, 0, 1]):
    pcd = o3d.geometry.PointCloud()
    #np_array = np.asarray(np_array)
    pcd.points = o3d.utility.Vector3dVector(np_array)
    pcd.paint_uniform_color(color)

    return pcd

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
    
    path_sub_10_7_9_45 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__10-07-2024_09-45-17", "Colon_subtriangles_2__10-07-2024_09-45-17_json.json")
    path_seg_compl_29_07_16_01 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__29-07-2024_16-15-42", "colon_segments_more_complicated__29-07-2024_16-15-42_json.json")
    path_sub_09_08_18_11 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__09-08-2024_18-11-41", "Colon_subtriangles_2__09-08-2024_18-11-41_json.json")
    path_haustren_09_08_15_22 = os.path.join(current_dir, "output_main", "4_colon_haustren_anim_text2__09-08-2024_15-28-28", "4_colon_haustren_anim_text2__09-08-2024_15-28-28_json.json")
    path_anim_haustren_color_16_10 = os.path.join(current_dir, "output_main", "4_colon_haustren_anim_text2_baked_color__16-10-2024_16-02-50", "4_colon_haustren_anim_text2_baked_color__16-10-2024_16-02-50_json.json")

    json_file_path = path_anim_haustren_color_16_10
    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)


        # extract data from json file
        dir_json = os.path.join (*input_liste["dir"])
        data_path = os.path.join(current_dir, *input_liste["data"])
        normals_to_inside = input_liste["normals_to_inside"]  
        medial_axis_bspline_path = os.path.join(current_dir, *input_liste["dir"],*input_liste["medial_axis_spline"])
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")
        mean_distance_point_to_point = input_liste["mean_distance_point_to_point"]
        #print(medial_axis_bspline_path)

        # read point clouds
        pcd_data = o3d.io.read_point_cloud(data_path)
        medial_axis_bspline = geomdl.exchange.import_json(medial_axis_bspline_path)[0]

        # get distance of points to mid_line & return all arrays
        #vector_to_line, t_on_line, vector_to_line_distances = find_min_distances_to_spline.get_closest_point_on_spline(pcd_data, medial_axis_bspline, normals_to_inside)
        vector_to_line, t_on_line, vector_to_line_distances = find_min_distances_to_spline.get_closest_point_on_spline_4(pcd_data, medial_axis_bspline, normals_to_inside, mean_distance_point_to_point)

        #arrow_plot(vector_to_line, pcd_data, np.asarray(medial_axis_bspline.evalpts))
        plot_vectors(t_on_line, vector_to_line_distances)
        plot_3d(vector_to_line, np.asarray(pcd_data.points),np.asarray(medial_axis_bspline.evalpts))

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