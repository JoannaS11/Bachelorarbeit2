import numpy as np
import open3d as o3d
import geomdl.BSpline
import geomdl.exchange
import os
import json

def convert_array_to_pcd(np_array, color):
    pcd = o3d.geometry.PointCloud()
    np_array = np.asarray(np_array)
    pcd.points = o3d.utility.Vector3dVector(np_array)
    pcd.paint_uniform_color(color)

    return pcd

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
            t_bigger_arg = np.argwhere (((t_on_line <= min_distances[i,0] + 0.5 * (min_distances[i + 1,0] - min_distances[i,0])) & (t_on_line >= min_distances[i,0])))
            part = pcd_data_np[np.r_[t_smaller_arg, t_bigger_arg]]
            part_pcd = convert_array_to_pcd(np.reshape(part, [np.shape(part)[0], np.shape(part)[-1]]), colors[k%divisor])
            vis.add_geometry(part_pcd)
            k+=1
        elif i == np.shape(min_distances)[0]-1:
            t_smaller_arg = np.argwhere(((t_on_line >= min_distances[i,0] - 0.5 * (min_distances[i,0]- min_distances[i-1,0])) & (t_on_line < min_distances[i,0])))
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

def plot_midline_as_pcd(pcd_data, midline, min_distances, b_spline):
    midline_pcd = convert_array_to_pcd(np.asarray(midline), [0, 0, 0])
    min_points = b_spline.evaluate_list(min_distances[:, 0])
    min_distances_pcd = convert_array_to_pcd(np.asarray(min_points), [1, 0, 0])
    #pcd_data.points = pcd_data.points[0:300]

    #pcd_data.shader = "defaultLitTransparency"
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultLitTransparency"
    mat.base_color = [0.0, 0.0, 1.0, 0.4]
    print("here")
    midline_pcd.paint_uniform_color([0,0,0])

    mat2 = o3d.visualization.rendering.MaterialRecord()
    mat2.base_color = [0.0, 0.0, 0.0, 0.2]

    o3d.visualization.draw([{'name':'colon', 'geometry':pcd_data, "material": mat},{'name': 'mid_line', 'geometry': midline_pcd, 'material':mat2}, {'name':'min_dist', 'geometry':min_distances_pcd}])
    pcd_data.paint_uniform_color([0.5,0.5,1])
    o3d.visualization.draw_geometries(
        [pcd_data],#, midline_pcd, min_distances_pcd],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
        point_show_normal=True,
    )

    o3d.visualization.draw_geometries(
        [midline_pcd, min_distances_pcd],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
    )

def plot_midline_and_pcd(pcd_data, midline, b_spline):
    midline_pcd = convert_array_to_pcd(np.asarray(midline), [0, 0, 0])
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultLitTransparency"
    mat.point_size = 1
    mat.base_color = [0.0, 0.0, 1.0, 0.4]
    mat2 = o3d.visualization.rendering.MaterialRecord()
    mat2.point_size=5
    midline_pcd.paint_uniform_color([0,0,0])
    #mat.albedo_img = o3d.io.read_image("../test_scripts/google_street.png")
    #pcd_data.material = mat

    o3d.visualization.draw([{'name': 'plane', 'geometry': pcd_data, 'material': mat}, {'name':'midline', 'geometry':midline_pcd, 'material':mat2}])

def plot_min_path_and_pcd(pcd_data, data_min_tree):
    midline_pcd = data_min_tree#convert_array_to_pcd(np.asarray(midline), [0, 0, 0])
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultLitTransparency"
    mat.point_size = 1
    mat.base_color = [0.0, 0.0, 1.0, 0.4]
    mat2 = o3d.visualization.rendering.MaterialRecord()
    mat2.point_size=5
    midline_pcd.paint_uniform_color([0,0,0])
    o3d.visualization.draw([{'name': 'plane', 'geometry': pcd_data, 'material': mat}, {'name':'midline', 'geometry':midline_pcd, 'material':mat2}])

def plot_smaller_pcd_and_pcd(pcd_data, pcd_smaller):
    plane = o3d.geometry.TriangleMesh.create_box(5,0.1,2, create_uv_map=True, map_texture_to_each_face=True)
    midline_pcd = pcd_smaller#convert_array_to_pcd(np.asarray(midline), [0, 0, 0])
    plane.compute_triangle_normals()
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultLitTransparency"
    mat.point_size = 1
    mat.base_color = [0.0, 0.0, 1.0, 0.4]
    mat2 = o3d.visualization.rendering.MaterialRecord()
    mat2.point_size=5
    midline_pcd.paint_uniform_color([0,0,0])
    #mat.albedo_img = o3d.io.read_image("../test_scripts/google_street.png")
    #pcd_data.material = mat

    o3d.visualization.draw([{'name': 'plane', 'geometry': pcd_data, 'material': mat}, {'name':'midline', 'geometry':midline_pcd, 'material':mat2}])



def main():
    current_dir = os.getcwd()

    path_anim_hausten_17_9_52 = os.path.join(current_dir, "output_main", "4_colon_haustren_anim_text2__17-07-2024_09-52-26", "4_colon_haustren_anim_text2__17-07-2024_09-52-26_json.json")
    path_sub_17_9_28 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__17-07-2024_09-28-00", "Colon_subtriangles_2__17-07-2024_09-28-00_json.json")
    path_seg_compl_29_07_16_15 = os.path.join(current_dir, "output_main", "colon_segments_more_complicated__29-07-2024_16-15-42", "colon_segments_more_complicated__29-07-2024_16-15-42_json.json")
    path_intestine = os.path.join("/home/yn86eniw/Documents/Bachelorarbeit2/output_main/intestine_short_texture_264000__27-09-2024_08-42-24/intestine_short_texture_264000__27-09-2024_08-42-24_json.json")
    path_anim_haustren_14_10 = os.path.join(current_dir, "output_main", "4_colon_haustren_anim_text2__14-10-2024_11-21-22", "4_colon_haustren_anim_text2__14-10-2024_11-21-22_json.json")
    path_intestine_14_10_265000 = os.path.join(current_dir, "output_main", "intestine_short_texture_264000__14-10-2024_09-29-26", "intestine_short_texture_264000__14-10-2024_09-29-26_json.json")
    path_sub_14_10 = os.path.join(current_dir, "output_main","Colon_subtriangles_2__14-10-2024_12-31-01", "Colon_subtriangles_2__14-10-2024_12-31-01_json.json")


    json_file_path = path_intestine_14_10_265000
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
        data_medial_pcd_path = os.path.join(current_dir, dir_json, *input_liste["medial_axis_big_pcd"])
        data_min_path = os.path.join(current_dir, dir_json, *input_liste["medial_axis_pcd"])

        # read point clouds
        pcd_data = o3d.io.read_point_cloud(data_path)
        medial_axis_bspline = geomdl.exchange.import_json(medial_axis_bspline_path)[0]
        pcd_medial_axis_big = o3d.io.read_point_cloud(data_medial_pcd_path)

        pcd_min_tree = o3d.io.read_point_cloud(data_min_path)

        # read motion array file
        motion_arrays = np.load(os.path.join(current_dir, dir_json, *t_on_line_path))
        t_on_line = motion_arrays["t_on_line"]
        vector_to_line = motion_arrays["vector_to_line"]
        vector_to_line_distances = motion_arrays["vector_to_line_distances"]
        min_distances = np.load(os.path.join(current_dir, dir_json, *local_min_path))
        min_distances = min_distances["local_mins"]

        print(f"Number of min Points: {np.shape(min_distances)[0]}")
        medial_axis_points = np.asarray(medial_axis_bspline.evalpts)
        plot_min_path_and_pcd(pcd_data, pcd_min_tree)
        plot_smaller_pcd_and_pcd(pcd_data, pcd_medial_axis_big)
        plot_midline_and_pcd(pcd_data, medial_axis_points, medial_axis_bspline)
        plot_midline_as_pcd(pcd_data, medial_axis_points, min_distances, medial_axis_bspline)

        plot_in_segments(np.asarray(pcd_data.points), t_on_line, min_distances, medial_axis_bspline)


if __name__ == "__main__":
    main()