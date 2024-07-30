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

"""def draw_geometries(pcds):
    """"""
    Draw Geometries
    Args:
        - pcds (): [pcd1,pcd2,...]
    """"""
    o3d.visualization.draw_geometries(pcds)

def get_o3d_FOR(origin=[0, 0, 0],size=10):
    """""" 
    Create a FOR that can be added to the open3d point cloud
    """"""
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=size)
    mesh_frame.translate(origin)
    return(mesh_frame)

def vector_magnitude(vec):
    """"""
    Calculates a vector's magnitude.
    Args:
        - vec (): 
    """"""
    magnitude = np.sqrt(np.sum(vec**2))
    return(magnitude)


def calculate_zy_rotation_for_arrow(vec):
    """"""
    Calculates the rotations required to go from the vector vec to the 
    z axis vector of the original FOR. The first rotation that is 
    calculated is over the z axis. This will leave the vector vec on the
    XZ plane. Then, the rotation over the y axis. 

    Returns the angles of rotation over axis z and y required to
    get the vector vec into the same orientation as axis z
    of the original FOR

    Args:
        - vec (): 
    """"""
    # Rotation over z axis of the FOR
    gamma = np.arctan(vec[1]/vec[0])
    Rz = np.array([[np.cos(gamma),-np.sin(gamma),0],
                   [np.sin(gamma),np.cos(gamma),0],
                   [0,0,1]])
    # Rotate vec to calculate next rotation
    vec = Rz.T@vec.reshape(-1,1)
    vec = vec.reshape(-1)
    # Rotation over y axis of the FOR
    beta = np.arctan(vec[0]/vec[2])
    Ry = np.array([[np.cos(beta),0,np.sin(beta)],
                   [0,1,0],
                   [-np.sin(beta),0,np.cos(beta)]])
    return(Rz, Ry)

def create_arrow(scale=10):
    """"""
    Create an arrow in for Open3D
    """"""
    cone_height = scale*0.2
    cylinder_height = scale*0.8
    cone_radius = scale/10
    cylinder_radius = scale/20
    mesh_frame = o3d.geometry.TriangleMesh.create_arrow(cone_radius=1,
        cone_height=cone_height,
        cylinder_radius=0.5,
        cylinder_height=cylinder_height)
    return(mesh_frame)

def get_arrow(origin=[0, 0, 0], end=None, vec=None):
    """"""
    Creates an arrow from an origin point to an end point,
    or create an arrow from a vector vec starting from origin.
    Args:
        - end (): End point. [x,y,z]
        - vec (): Vector. [i,j,k]
    """"""
    scale = 10
    Ry = Rz = np.eye(3)
    T = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T[:3, -1] = origin
    if end is not None:
        vec = np.array(end) - np.array(origin)
    elif vec is not None:
        vec = np.array(vec)
    if end is not None or vec is not None:
        scale = vector_magnitude(vec)
        Rz, Ry = calculate_zy_rotation_for_arrow(vec)
    mesh = create_arrow(scale)
    # Create the arrow
    mesh.rotate(Ry, center=np.array([0, 0, 0]))
    mesh.rotate(Rz, center=np.array([0, 0, 0]))
    mesh.translate(origin)
    return(mesh)

def arrow_plot(vector_to_line, pcd_data, mid_points):
    mid_pc = convert_array_to_pcd(mid_points, [0,0,0])
    data_np = np.asarray(pcd_data.points)
    # Create a Cartesian Frame of Reference
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    FOR = get_o3d_FOR()
    # Create an arrow from point (5,5,5) to point (10,10,10)
    # arrow = get_arrow([5,5,5],[10,10,10])
    vis.add_geometry(mid_pc)
    for i in range(np.shape(vector_to_line)[0]):
    # Create an arrow representing vector vec, starting at (5,5,5)
        arrow = get_arrow(data_np[i,:],vec=vector_to_line[i,:])

        # Create an arrow in the same place as the z axis
        arrow = get_arrow()

        # Draw everything
        vis.add_geometry([FOR,arrow])"""



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

    #fig = plt.figure()
    print("hallo")

    """ax = fig.add_subplot(111, projection='3d')
    ax.quiver(pcd_data[50000:100000,0], pcd_data[50000:100000,1], pcd_data[50000:100000,2], vector_to_line[50000:100000,0], vector_to_line[50000:100000,1], vector_to_line[50000:100000,2], color='g')
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
    fig = plt.figure()
    print("hallo")

    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(pcd_data[100000:,0], pcd_data[100000:,1], pcd_data[100000:,2], vector_to_line[100000:,0], vector_to_line[100000:,1], vector_to_line[100000:,2], color='g')
    #ax.quiver(pcd_colon[1000:,0], pcd_colon[1000:,1], pcd_colon[1000:,2], vector_to_line[1000:,0], vector_to_line[1000:,1], vector_to_line[1000:,2], color='b')
    #ax.plot(start_points[:,0], start_points[:,1], start_points[:,2], color = 'r')
    ax.scatter(mid_line[:,0], mid_line[:,1], mid_line[:,2], color = 'c')
    #ax.scatter(pcd_colon[759,0],pcd_colon[759,1], pcd_colon[759,2], color = 'b')
    #ax.scatter(l[0],l[1], l[2], color = 'b')
    # Set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()"""

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


    json_file_path = path_seg_compl_29_07_16_01
    with open(json_file_path, 'r+') as input_file:
        input_liste = json.load(input_file)


        # extract data from json file
        dir_json = os.path.join (*input_liste["dir"])
        data_path = os.path.join(current_dir, *input_liste["data"])
        normals_to_inside = input_liste["normals_to_inside"]  
        medial_axis_bspline_path = os.path.join(current_dir, *input_liste["dir"],*input_liste["medial_axis_spline"])
        data_name = input_liste["data"][-1]
        data_name = data_name.replace(".ply", "")
        #print(medial_axis_bspline_path)

        # read point clouds
        pcd_data = o3d.io.read_point_cloud(data_path)
        medial_axis_bspline = geomdl.exchange.import_json(medial_axis_bspline_path)[0]

        print("after import")

        # get distance of points to mid_line & return all arrays
        vector_to_line, t_on_line, vector_to_line_distances = find_min_distances_to_spline.get_closest_point_on_spline(pcd_data, medial_axis_bspline, normals_to_inside)

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