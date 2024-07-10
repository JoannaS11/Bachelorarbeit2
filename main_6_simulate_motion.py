import json
import geomdl.BSpline
import geomdl.exchange
import f_simulate_motion
import os
import open3d as o3d
import numpy as np
import geomdl
import matplotlib.pyplot as plt


def convert_array_to_pcd(np_array, color):
    pcd = o3d.geometry.PointCloud()
    #np_array = np.asarray(np_array)
    pcd.points = o3d.utility.Vector3dVector(np_array)
    pcd.paint_uniform_color(color)

    return pcd


def plot_midline_distances(pcd_data, midline, min_distances):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    midline_np = np.asarray(midline)
    pcd_data_np = np.asarray(pcd_data)
    print(midline_np.shape)
    ax.scatter(midline_np[:, 0], midline_np[:, 1], midline_np[:, 2], color="b")
    ax.scatter(
        pcd_data_np[:, 0], pcd_data_np[:, 1], pcd_data_np[:, 2], color="c", alpha=0.1
    )
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    plt.show()


def plot_midline_as_pcd(pcd_data, midline, min_distances, b_spline):
    midline_pcd = convert_array_to_pcd(np.asarray(midline), [0, 0, 0])
    min_points = b_spline.evaluate_list(min_distances[:, 0])
    min_distances_pcd = convert_array_to_pcd(np.asarray(min_points), [1, 0, 0])

    #pcd_data.shader = "defaultLitTransparency"
    o3d.visualization.draw_geometries(
        [pcd_data, midline_pcd, min_distances_pcd],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
        point_show_normal=True,
    )

    o3d.visualization.draw_geometries(
        [midline_pcd, min_distances_pcd],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
    )
6

def main():
    current_dir = os.getcwd()
    # json file paths
    path_colon_sub = os.path.join(current_dir,"output_main","Colon_subtriangles_2__02-07-2024_11-22-47","Colon_subtriangles_2_02-07-2024_11-22-47_json.json")#gut
    path_zyl_compl_4 = os.path.join(current_dir,"output_main","zylinder_compl-4__02-07-2024_10-40-05","zylinder_compl-4_02-07-2024_10-40-05_json.json")
    path_zyl_compl_2 = os.path.join(current_dir,"output_main","zylinder_compl-2__02-07-2024_11-03-02","zylinder_compl-2_02-07-2024_11-03-02_json.json")
    path_seg = os.path.join(current_dir,"output_main","colon_segments__02-07-2024_11-06-21","colon_segments_02-07-2024_11-06-21_json.json")
    path_seg_compl = os.path.join(current_dir,"output_main","colon_segments_more_complicated__02-07-2024_11-09-37","colon_segments_more_complicated_02-07-2024_11-09-37_json.json")#gut
    path_in_sh_tex = os.path.join(current_dir,"output_main","intestine_short_texture_anim__02-07-2024_13-24-24","intestine_short_texture_anim__02-07-2024_13-24-24_json.json")
    path_anim_hausten = os.path.join(current_dir,"output_main", "4_colon_haustren_anim_text2__03-07-2024_08-16-41", "4_colon_haustren_anim_text2__03-07-2024_08-16-41_json.json")
    path_colon_sub_08_07 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__08-07-2024_15-47-05","Colon_subtriangles_2__08-07-2024_15-47-05_json.json")
    path_seg_compl_8 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__08-07-2024_13-18-56", "colon_segments_more_complicated__08-07-2024_13-18-56_json.json")
    path_seg_compl_9_15_15 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__09-07-2024_15-14-03", "colon_segments_more_complicated__09-07-2024_15-14-03_json.json")
    path_colon_sub_09_07 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__09-07-2024_15-17-15", "Colon_subtriangles_2__09-07-2024_15-17-15_json.json")
    path_seg_compl_10_9_39 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__10-07-2024_09-37-50", "colon_segments_more_complicated__10-07-2024_09-37-50_json.json")
    path_sub_10_09_45 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__10-07-2024_09-45-17", "Colon_subtriangles_2__10-07-2024_09-45-17_json.json")


    json_file_path = path_sub_10_09_45
    with open(json_file_path, "r+") as input_file:
        input_liste = json.load(input_file)

        # extract data from json file
        dir_json = os.path.join(*input_liste["dir"])
        data_path = os.path.join(current_dir, *input_liste["data"])
        medial_axis_bspline_path = os.path.join(
            current_dir, *input_liste["dir"], *input_liste["medial_axis_spline"]
        )
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")
        t_on_line_path = input_liste["t_on_line"][1:]
        local_min_path = input_liste["min_distances_values"][1:]

        # read point clouds
        pcd_data = o3d.io.read_point_cloud(data_path)
        medial_axis_bspline = geomdl.exchange.import_json(medial_axis_bspline_path)[0]

        # read motion array file
        motion_arrays = np.load(os.path.join(current_dir, dir_json, *t_on_line_path))
        t_on_line = motion_arrays["t_on_line"]
        vector_to_line = motion_arrays["vector_to_line"]
        min_distances = np.load(os.path.join(current_dir, dir_json, *local_min_path))
        min_distances = min_distances["local_mins"]
        
        print("after import")

        # show the different results
        """plot_midline_distances(
            pcd_data.points, medial_axis_bspline.evalpts, min_distances
        )"""
        print(f"Number of min Points: {np.shape(min_distances)[0]}")
        medial_axis_points = np.asarray(medial_axis_bspline.evalpts)
        plot_midline_as_pcd(pcd_data, medial_axis_points, min_distances, medial_axis_bspline)

        # simulate motion
        f_simulate_motion.simulate_motion(
            medial_axis_bspline, pcd_data, min_distances, vector_to_line, t_on_line
        )


if __name__ == "__main__":
    main()
