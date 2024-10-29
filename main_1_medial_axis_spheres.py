import medial_axis_spheres
import numpy as np
import os
from datetime import datetime
import open3d as o3d
import json
import shutil


def export_pcd_as_ply(pcd, path, output_name_without_ply):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    name = f"{date_time}_{output_name_without_ply}.ply"
    o3d.io.write_point_cloud(
        os.path.join(path, f"{date_time}_{output_name_without_ply}.ply"), pcd
    )
    return name


def find_smaller_pcd_data(
    pcd_data, zyl_points, zyl_normals, mini_residual, normals_inside, dir_name
):
    # adapt normals
    if normals_inside:
        pass
    else:
        zyl_normals = -zyl_normals

    # get big line pointclouds
    (
        pcd_big_line_with,
        pcd_big_line_without,
        mean_distance_point_point,
        mean_distance_point_to_line,
    ) = medial_axis_spheres.get_big_line_pointcloud(
        pcd_data, zyl_points, zyl_normals, mini_residual
    )

    # export as ply
    filename = f"_{mean_distance_point_point}_{mean_distance_point_to_line}_{mini_residual}_pcd_data_big_path_without_outlier"
    export_pcd_as_ply(
        pcd_big_line_with,
        dir_name,
        f"_{mean_distance_point_point}_{mean_distance_point_to_line}_{mini_residual}_pcd_data_big_path",
    )
    name_with = export_pcd_as_ply(
        pcd_big_line_with,
        dir_name,
        f"_{mean_distance_point_point}_{mean_distance_point_to_line}_{mini_residual}_pcd_data_big_path",
    )
    name_without = export_pcd_as_ply(pcd_big_line_without, dir_name, filename)

    colors_2 = np.asarray(pcd_big_line_with.colors)
    colors_2[0:100] = [0, 0, 0]
    colors_2[100:] = [1, 0, 1]

    pcd_big_line_with.colors = o3d.utility.Vector3dVector(colors_2)
    pcd_data.paint_uniform_color([1, 1, 0])

    # visualize
    o3d.visualization.draw_geometries(
        [pcd_data, pcd_big_line_with],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
        point_show_normal=True,
    )
    return (
        pcd_big_line_with,
        name_with,
        pcd_big_line_without,
        name_without,
        mean_distance_point_point,
        mean_distance_point_to_line,
    )


# def plot_line_pcds(pcd_big_line_2, pcd_data):


def main():
    current_dir = os.getcwd()
    template_json = os.path.join(
        current_dir, "output_main", "empty_object_paths_json.json"
    )
    ################################ data paths ####################################################################
    path_z_complex_4 = "zylinder_compl-4.ply"
    path_z_complex_2 = "zylinder_compl-2.ply"
    path_z_simple = "zylinder_simple.ply"
    path_colon = "Colon.ply"
    path_colon_seg = "colon_segments.ply"
    path_colon_seg_compl = "colon_segments_more_complicated.ply"
    path_intestine_short_texture_anim = "intestine_short_texture_anim.ply"
    path_subtriangles_2 = "Colon_subtriangles_2.ply"
    path_anim_haustren = "4_colon_haustren_anim_text2.ply"

    object_name = path_colon_seg_compl
    path = os.path.join(current_dir, "data", object_name)

    path = "output_main/point_cloud_with_normals__10-09-2024_13-18-48/10-09-2024_13-20-14__0.426151016746085_0.09454614150339564_30_pcd_data_big_path_without_outlier.ply"
    object_name = "point_cloud_with_normals.ply"
    # load point clouds
    pcd_data = o3d.io.read_point_cloud(path)

    ################################# create directory to save results ###########################################
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    object_name_without_ply = object_name.replace(".ply", "_")
    dir_name_end = f"{object_name_without_ply}_{date_time}"
    dir_name = os.path.join(current_dir, "output_main", dir_name_end)
    os.mkdir(dir_name)

    ################################## create json file ###########################################################
    json_file_path = os.path.join(
        dir_name, f"{object_name_without_ply}_{date_time}_json.json"
    )
    open(json_file_path, "w")
    shutil.copyfile(template_json, json_file_path)

    ################################ medial-axis-spheres ##########################################################
    # get points and normals
    zyl_normals = np.asarray(pcd_data.normals)
    zyl_points = np.asarray(pcd_data.points)

    # visualize to see direction of normals
    o3d.visualization.draw_geometries(
        [pcd_data],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
        point_show_normal=True,
    )

    normals_to_inside_answer = input(
        "Are normals pointing to inside? yes: y, no: insert n:  "
    )
    if normals_to_inside_answer == "y":
        normals_to_inside = True
    elif normals_to_inside_answer == "n":
        normals_to_inside = False
    else:
        raise Exception("wrong input- insert either y or n")

    with open(json_file_path, "r+") as input_file:
        input_liste = json.load(input_file)

        # write normals_inside to json file
        input_liste["normals_to_inside"] = normals_to_inside

        # write data_path to json_file
        input_liste["data"] = ["data", object_name]

        # write dir to json_file
        input_liste["dir"] = ["output_main", dir_name_end]

        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)

        mini_residual = int(np.max([2, np.shape(zyl_points)[0] // 850]))
        input_liste["mini_residual"] = mini_residual

        (
            pcd_big_line,
            pcd_big_line_path,
            pcd_big_line_without,
            pcd_big_line_without_path,
            mean_distance_point_point,
            max_distance_point_to_line,
        ) = find_smaller_pcd_data(
            pcd_data,
            zyl_points,
            zyl_normals,
            mini_residual,
            normals_to_inside,
            dir_name,
        )
        input_liste["medial_axis_big_pcd"] = [pcd_big_line_path]
        input_liste["medial_axis_big_pcd_without_outlier"] = [pcd_big_line_without_path]
        input_liste["mean_distance_point_to_point"] = mean_distance_point_point
        input_liste["distance_point_to_line"] = max_distance_point_to_line

        input_file.seek(0)
        json.dump(input_liste, input_file, indent=4)


if __name__ == "__main__":
    main()
