import json
import geomdl.BSpline
import geomdl.exchange
import f_simulate_motion_multiple as f_simulate_motion_multiple
import f_simulate_motion_multiple_peristaltic
import f_simulate_motion_multiple_mass_movement
import f_simulate_motion_peristalsis_without_min
import os
import open3d as o3d
import numpy as np
import geomdl
import matplotlib.pyplot as plt

def plot_in_segments(pcd_data_np, t_on_line, min_distances, bspline):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    midline = convert_array_to_pcd(np.asarray(bspline.evalpts), [0,0,0])
    vis.add_geometry(midline)
    colors = np.array([[1,0,0],[0,0,1], [1,0,1], [1,1,0], [0, 1, 1], [0.5,0,0.5]])
    divisor = 6
    k = 0
    for i in range(np.shape(min_distances)[0]):
        if i == 0:
            t_smaller_arg = np.argwhere(t_on_line < min_distances[i,0])
            t_bigger_arg = np.argwhere (((t_on_line <= min_distances[i,0] + 0.5 * (min_distances[i + 1,0]- min_distances[i,0])) & (t_on_line >= min_distances[i,0])))
            part = pcd_data_np[np.r_[t_smaller_arg, t_bigger_arg]]
            part_pcd = convert_array_to_pcd(np.reshape(part, [np.shape(part)[0], np.shape(part)[-1]]), colors[k%divisor])
            vis.add_geometry(part_pcd)
            k+=1
        elif i == np.shape(min_distances)[0]-1:
            t_smaller_arg = np.argwhere(((t_on_line > min_distances[i,0] - 0.5 * (min_distances[i,0]- min_distances[i-1,0])) & (t_on_line < min_distances[i,0])))
            t_bigger_arg = np.argwhere ((t_on_line >= min_distances[i,0]))
            part = pcd_data_np[np.r_[t_smaller_arg, t_bigger_arg]]
            part_pcd = convert_array_to_pcd(np.reshape(part, [np.shape(part)[0], np.shape(part)[-1]]), colors[k%divisor])
            vis.add_geometry(part_pcd)
            k+=1
        else:
            t_smaller_arg = np.argwhere(((t_on_line > (min_distances[i,0] - 0.5 * (min_distances[i,0]- min_distances[i-1,0]))) & (t_on_line < min_distances[i,0])))
            t_bigger_arg = np.argwhere (((t_on_line <= min_distances[i,0] + 0.5 * (min_distances[i + 1,0] - min_distances[i,0])) & (t_on_line >= min_distances[i,0])))
            part = pcd_data_np[np.r_[t_smaller_arg, t_bigger_arg]]
            part_pcd = convert_array_to_pcd(np.reshape(part, [np.shape(part)[0], np.shape(part)[-1]]), colors[k%divisor])
            vis.add_geometry(part_pcd)
            k+=1
    vis.run()
    vis.destroy_window()


def plot_in_segments(pcd_data_np, t_on_line, min_distances, bspline):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    midline = convert_array_to_pcd(np.asarray(bspline.evalpts), [0,0,0])
    vis.add_geometry(midline)
    colors = np.array([[1,0,0],[0,0,1], [1,0,1], [1,1,0], [0, 1, 1], [0.5,0,0.5]])
    divisor = 6
    k = 0
    for i in range(np.shape(min_distances)[0]):
        if i == 0:
            t_smaller_arg = np.argwhere(t_on_line < min_distances[i,0])
            t_bigger_arg = np.argwhere (((t_on_line <= min_distances[i,0] + 0.5 * (min_distances[i + 1,0]- min_distances[i,0])) & (t_on_line >= min_distances[i,0])))
            part = pcd_data_np[np.r_[t_smaller_arg, t_bigger_arg]]
            part_pcd = convert_array_to_pcd(np.reshape(part, [np.shape(part)[0], np.shape(part)[-1]]), colors[k%divisor])
            vis.add_geometry(part_pcd)


            k+=1
        elif i == np.shape(min_distances)[0]-1:
            t_smaller_arg = np.argwhere(((t_on_line > min_distances[i,0] - 0.5 * (min_distances[i,0]- min_distances[i-1,0])) & (t_on_line < min_distances[i,0])))
            t_bigger_arg = np.argwhere ((t_on_line >= min_distances[i,0]))
            part = pcd_data_np[np.r_[t_smaller_arg, t_bigger_arg]]
            part_pcd = convert_array_to_pcd(np.reshape(part, [np.shape(part)[0], np.shape(part)[-1]]), colors[k%divisor])
            vis.add_geometry(part_pcd)

            k+=1
        else:
            t_smaller_arg = np.argwhere(((t_on_line >= (min_distances[i,0] - 0.5 * (min_distances[i,0]- min_distances[i-1,0]))) & (t_on_line < min_distances[i,0])))
            t_bigger_arg = np.argwhere (((t_on_line <= min_distances[i,0] + 0.5 * (min_distances[i + 1,0] - min_distances[i,0])) & (t_on_line >= min_distances[i,0])))


            part = pcd_data_np[np.r_[t_smaller_arg, t_bigger_arg]]
            part_pcd = convert_array_to_pcd(np.reshape(part, [np.shape(part)[0], np.shape(part)[-1]]), colors[k%divisor])
            vis.add_geometry(part_pcd)

            k+=1
    vis.run()
    vis.destroy_window()


def convert_array_to_pcd(np_array, color):
    pcd = o3d.geometry.PointCloud()
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


def main():
    current_dir = os.getcwd()
    # json file paths
    path_colon_subtr_22_10 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__22-10-2024_08-19-37", "Colon_subtriangles_2__22-10-2024_08-19-37_json.json")
    path_colon_intestine_22_10_265000 = os.path.join(current_dir, "output_main", "intestine_short_texture_264000__22-10-2024_07-52-39", "intestine_short_texture_264000__22-10-2024_07-52-39_json.json")
    path_anim_haustrae_color_22_10 = os.path.join(current_dir, "output_main", "4_colon_haustren_anim_text2_baked_color__22-10-2024_10-42-30", "4_colon_haustren_anim_text2_baked_color__22-10-2024_10-42-30_json.json")

    path_intestine_28_10_16577 = os.path.join(current_dir, "output_main", "intestine_short_texture_anim_16577__28-10-2024_14-12-17", "intestine_short_texture_anim_16577__28-10-2024_14-12-17_json.json")
    path_intestine_28_10_66177 = os.path.join(current_dir, "output_main", "intestine_short_texture_colon_66177_1_10_1__28-10-2024_13-02-01", "intestine_short_texture_colon_66177_1_10_1__28-10-2024_13-02-01_json.json")
    path_intestine_28_10_264000 = os.path.join(current_dir, "output_main", "intestine_short_texture_264000__28-10-2024_13-05-57", "intestine_short_texture_264000__28-10-2024_13-05-57_json.json")
    path_intestine_28_10_1057000 = os.path.join(current_dir, "output_main", "intestine_short_texture_anim_more_points__28-10-2024_13-15-31", "intestine_short_texture_anim_more_points__28-10-2024_13-15-31_json.json")



    json_file_path = path_intestine_28_10_1057000
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
        vector_to_line_distances = motion_arrays["vector_to_line_distances"]
        min_distances = np.load(os.path.join(current_dir, dir_json, *local_min_path))
        min_distances = min_distances["local_mins"]
        
        print("after import")
        plot_in_segments(np.asarray(pcd_data.points), t_on_line, min_distances, medial_axis_bspline)

        speed = 0.026
        reverse = False

        #f_simulate_motion_peristalsis_without_min.simulate_motion_parallel(pcd_data, vector_to_line, t_on_line)
        f_simulate_motion_multiple_mass_movement.simulate_motion_parallel_2(
            medial_axis_bspline, pcd_data, min_distances, vector_to_line, t_on_line, speed, reverse
        )
        f_simulate_motion_multiple_peristaltic.simulate_motion_parallel_2(
            medial_axis_bspline, pcd_data, min_distances, vector_to_line, t_on_line, speed
        )
        f_simulate_motion_multiple.simulate_motion_parallel_2(
            medial_axis_bspline, pcd_data, min_distances, vector_to_line, t_on_line, speed
        )


if __name__ == "__main__":
    main()
