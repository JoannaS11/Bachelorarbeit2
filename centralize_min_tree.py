import numpy as np
import open3d as o3d
from datetime import datetime
import os


def get_point_before_and_behind(pcd_colon, min_path_point, ortho, pcd_colon_normals, distance_point_to_line):
    behind_points_list = [0,0,0]
    before_points_list = [0,0,0]

    # calculate distance from points to line(startpoint + normal)
    distance = np.ndarray(np.shape(pcd_colon)[0])
    tmp = np.cross((pcd_colon[:]- min_path_point), ortho)
    for x in range(np.shape(pcd_colon)[0]):
        distance[x] = np.sqrt(tmp[x][0] ** 2 + tmp[x][1] ** 2 + tmp[x][2] ** 2) / np.linalg.norm(ortho)

    # get distance indices within threshold from line
    #distance[line_start] = 100
    pos_min = np.argwhere(abs(distance) < distance_point_to_line)

    # calculate distance from each of the points to startpoint        
    points_min = pcd_colon[pos_min[:]]
    vector_min = points_min[:]- min_path_point
    distance_min = np.ndarray([np.shape(points_min)[0], 1])
    for i in range(np.shape(distance_min)[0]):
        distance_min[i] = np.sqrt(np.dot(np.reshape(vector_min[i][0], [1,3]) , np.reshape(vector_min[i][0], [3,1])))

    # sort distances
    sort_indices = np.argsort(distance_min, axis = 0)
    pos_min = pos_min[sort_indices[:]]

    before_point = False
    behind_point = False
    for index in pos_min:
        if before_point and behind_point:
            break
        # check that point is not behind the point
        if np.dot(pcd_colon[index] - min_path_point, - ortho) < 0:
                # check that normals look in contrary directions
            if np.dot(ortho, pcd_colon_normals[index[0,0]]) < 0:
                before_points_list = pcd_colon[index[0,0]]
                before_point = True
        else:
            if np.dot(ortho, pcd_colon_normals[index[0,0]]) > 0:
                behind_points_list = pcd_colon[index[0,0]]
                behind_point = True
    
    return before_points_list, behind_points_list

    

def centralize_min_tree(min_path, pcd_colon, pcd_colon_normals, distance_point_to_line):
    new_min_path = np.zeros(np.shape(min_path))
    print(pcd_colon_normals.shape)
    for i in range(1,np.shape(min_path)[0]):
        gradient = min_path[i] - min_path[i - 1]
        ortho_2 = np.array([1.0,1.0,0.0])
        ortho_2 -= ortho_2.dot(gradient) * gradient  
        ortho_2 /= np.linalg.norm(ortho_2)
        ortho_1 = np.cross(gradient, ortho_2)
        
        if i == 1:
            before_points_liste, behind_points_liste = get_point_before_and_behind(pcd_colon, min_path[0], ortho_1, pcd_colon_normals, distance_point_to_line)
            new_midpoint = before_points_liste + 0.5 *  (behind_points_liste - before_points_liste)

            before_points_liste, behind_points_liste = get_point_before_and_behind(pcd_colon, new_midpoint, ortho_2, pcd_colon_normals, distance_point_to_line)

            new_min_path[0] = before_points_liste + 0.5 *  (behind_points_liste - before_points_liste)

        before_points_liste, behind_points_liste = get_point_before_and_behind(pcd_colon, min_path[i], ortho_1, pcd_colon_normals, distance_point_to_line)
        new_midpoint = before_points_liste + 0.5 *  (behind_points_liste - before_points_liste)

        before_points_liste, behind_points_liste = get_point_before_and_behind(pcd_colon, new_midpoint, ortho_2, pcd_colon_normals, distance_point_to_line)

        new_min_path[i] = before_points_liste + 0.5 *  (behind_points_liste - before_points_liste)
        
    return new_min_path

def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name = None):
    # get current date and time
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    output_name_without_ply = output_name_without_ply.replace(".", "-")
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{output_name_without_ply}__at_{date_time}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), f"{output_name_without_ply}__at_{date_time}.ply"), pcd)     

def main():
    ######### path ################
    # prepare colon for visualization
    path_z_simple = os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_2= os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_4 = os.path.join(os.getcwd(), "data","zylinder_compl-4.ply")
    path_colon = os.path.join(os.getcwd(), "data/Colon.ply")

    path_subtriangles_2 = os.path.join(os.getcwd(), "data","Colon_subtriangles_2.ply")

    pcd_colon_input = o3d.io.read_point_cloud(path_z_compl_4)

    pcd_colon = o3d.io.read_point_cloud(path_z_compl_4)
    pcd_colon.paint_uniform_color([1,1,0])
    
    # data paths
    path_ = os.path.join(os.getcwd(), "output_new","2024-05-27_08-22-28-505194_line_2.ply")
    path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-28_16-13-50-420242_0.25_min_path.ply")
    path_colon_min = os.path.join(os.getcwd(), "output_new", "2024-06-03_10-23-22-088632_0.425_min_path.ply")#"2024-06-03_10-24-25-169883_0.4_min_path.ply")
    path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-07_16-01-05-609625_0.4_min_path.ply")
    path_zyl_compl_2 = os.path.join(os.getcwd(), "output_new", "2024-06-11_10-45-54-282186_0.4_min_path.ply")
    path_zyl_compl_4 = os.path.join(os.getcwd(), "output_new", "2024-06-11_14-41-33-528331_0.4_min_path.ply")#

    path_zyl_compl_4_curve = os.path.join(os.getcwd(), "output_curve", "2024-06-16_10-44-31-592601_curve_as_pointcloud-2.ply")#

    path_min = path_zyl_compl_4_curve
    pcd = o3d.io.read_point_cloud(path_min)
    pcd.paint_uniform_color([0,0,1])
    min_path = np.asarray(pcd.points)

    # changeable parameter
    distance_point_to_line = 0.4


    new_min_path = centralize_min_tree(min_path, np.asarray(pcd_colon_input.points), np.asarray(pcd_colon_input.normals), distance_point_to_line)

    # convert to pcd
    line = o3d.geometry.PointCloud()
    line.points = o3d.utility.Vector3dVector(new_min_path)
    line.paint_uniform_color([1,0,0])


    file_name = f"{path_min}_{distance_point_to_line}_centralized"
    file_name = file_name.replace(".ply","_")
    file_name = file_name.replace(os.getcwd(),"")
    export_pcd_as_ply(line, "", file_name, dir_name = None)

    # visualize
    o3d.visualization.draw_geometries([pcd_colon, line, pcd], mesh_show_wireframe = True, mesh_show_back_face = True, point_show_normal = True)



if __name__=="__main__": main()