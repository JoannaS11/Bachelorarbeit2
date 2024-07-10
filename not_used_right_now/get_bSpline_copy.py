import geomdl.fitting
import numpy as np
import geomdl
import os
import open3d as o3d
import scipy
import matplotlib.pyplot as plt
import scipy.interpolate
from datetime import datetime
from time import sleep
import math
import not_used_right_now.plot_file as plot_file

import copy 

def get_bSpline(pcd, sample_size):
    pcd_np = np.asarray(pcd.points)

    # interpolate line and upsample
    line_pc_array = np.ndarray.tolist(pcd_np)

    b_spline = geomdl.fitting.interpolate_curve(line_pc_array, 2)
    b_spline.sample_size = sample_size * b_spline.sample_size

    return b_spline


def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name = None):
    # get current date and time
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    # export as ply
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"), pcd)


def convert_bSpline_to_pcd(bSpline):
    # get points from bSpline
    points_bSpline = bSpline.evalpts
    points_bSpline_np = np.array(points_bSpline)

    # create point cloud from curve
    line_pcd = o3d.geometry.PointCloud()    
    line_pcd.points = o3d.utility.Vector3dVector(points_bSpline_np)

    return line_pcd


def visualize(pcd_colon):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    for i in range(100):
        pcd_colon.paint_uniform_color(change_color()[0])
        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(1)

    vis.register_animation_callback(move_colon)
    vis.run()
    vis.destroy_window()


def change_color():
    color = np.random.rand(1,3)
    print(color)
    
    return color
    #vis.get_view_Control()


def move_colon(vis):
    color = np.random.rand([3,1])

    #vis.get_view_Control()

def get_closest_point_on_spline(pcd, bSpline):
    bSpline.evaluate()
    #print(f"splijne {len(bSpline)} and {bSpline.evaluate_single(0.501)}")
    points = np.asarray(pcd.points)
    #t_old = 0.1
    #p = points[4000]
    #print(p[1])
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    half_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])
    #print(f" {np.linspace(0, 1, 20)} and {type(np.linspace(0, 1, 20))}")
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, 40)))
    print(start_points)
    for i in range(np.shape(points)[0]):
        # approximate start value to be determined
        p_min = 0
        p_dist = 1000
        p = points[i]
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])
            if i == 757 or i == 759:
                print(f"i: {i} p_dist{p_dist} and dist {dist} all")
            if dist[po] < p_dist:
                p_dist = dist[po]
                p_min = po
                if i == 757 or i == 759:
                    print(f"i: {i} p_dist {p_dist} and p_min {p_min} who mad it")
        
        t_old = np.linspace(0, 1, 40)[p_min]

        for k in range(30):
            #print(t_old)
            point_on_spline = bSpline.derivatives(t_old, 2)
            #print(point_old_2)
            f_t = (point_on_spline[0][0] - p[0]) * point_on_spline[1][0] + (point_on_spline[0][1] - p[1]) * point_on_spline[1][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[1][2] 
            f__t = point_on_spline[1][0] * point_on_spline[1][0] + point_on_spline[1][1] * point_on_spline[1][1] + point_on_spline[1][2] * point_on_spline[1][2]#+ (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1]#+ point_on_spline[1][2] * point_on_spline[1][2] + (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[2][2]
            t = t_old - f_t / f__t    
   
            if t < 0.00:
                t_old = 0.0
            elif t > 1.00:
                t_old = 1.0
            else:
                t_old = t  

        vector_to_line[i] = bSpline.evaluate_single(t_old) - p
        t_on_line[i] = t_old
        #
        # sleep(0.5)
        #print(vector_to_line)

        half_line[i] = p + 0.3 * vector_to_line[i]
        #print(half_line[i])

    x = np.argwhere(t_on_line > 0.5)
    #print(x)
    #x = np.argwhere(t_on_line[x[:]] > 0.0000)
    #
    #print(t_on_line[0:1000])
    #vector_to_line[x[:]] = 0
    #[x[:]] = points[x[:]] + 0.1 * vector_to_line[x[:]]
    l = bSpline.evaluate_single(0.042383170524102884)
    plot_file.plot_vectors(vector_to_line, points, start_points,l)
    #half_line = points + 0.5 * vector_to_line

    
    
    return vector_to_line, t_on_line, half_line


def plot_vectors(vector_to_line, pcd_colon, start_points):
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')

    ax.quiver(pcd_colon[:,0], pcd_colon[:,1], pcd_colon[:,2], vector_to_line[:,0], vector_to_line[:,1], vector_to_line[:,2])
    ax.plot(*start_points, color = 'r')
    # Set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()


def find_smallest_dis_to_point(pcd_colon, bspline, vector_to_line, t_on_line):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    print("there")
    points = copy.deepcopy(np.asarray(pcd_colon.points))
    z = 0
    # copy by reference
    half_line = np.asarray(pcd_colon.points)
    print("h")
    for i in range(500):
        dist_to_z = np.power(np.abs(1 - np.abs((z % 1.0) - t_on_line[:])), 4)
        dist_to_z = np.power(np.abs(1 - np.abs((z % 1.0) - t_on_line[:])), 4)
        #print(f"dist to z {dist_to_z}")
        
        #disc = np.argwhere(np.abs((z % 1.0) - t_on_line[:]) > 0.5)
        #print(f"shkpae {np.shape(disc)}")
        half_line[:, 0] = points[:, 0] + np.multiply(dist_to_z, vector_to_line[:, 0])
        half_line[:, 1] = points[:, 1] + np.multiply(dist_to_z, vector_to_line[:, 1])
        half_line[:, 2] = points[:, 2] + np.multiply(dist_to_z, vector_to_line[:, 2])
        #half_line[10] = points[10] + 0.25 * vector_to_line[10]
        #pcd_colon.points = o3d.utility.Vector3dVector(half_line)

        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(0.1)
        z+=0.01


        """half_line = points + 0.1* vector_to_line
        
        #half_line[10] = points[10] + 0.25 * vector_to_line[10]
        pcd_colon.points = o3d.utility.Vector3dVector(half_line)

        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(0.1)
        z+=0.001"""


    #vis.register_animation_callback(move_colon)
    vis.run()
    vis.destroy_window()

def main():

    # adjustable parameter
    sample_size = 2

    # prepare colon for visualization
    path_z_simple = os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_2= os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_4 = os.path.join(os.getcwd(), "data","zylinder_compl-4.ply")
    path_colon = os.path.join(os.getcwd(), "data/Colon.ply")
    pcd_colon = o3d.io.read_point_cloud(path_colon)
    pcd_colon.paint_uniform_color([1,1,0])
    
    # data paths
    path_ = os.path.join(os.getcwd(), "output_new","2024-05-27_08-22-28-505194_line_2.ply")
    path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-28_16-13-50-420242_0.25_min_path.ply")
    path_colon_min = os.path.join(os.getcwd(), "output_new", "2024-06-03_10-23-22-088632_0.425_min_path.ply")#"2024-06-03_10-24-25-169883_0.4_min_path.ply")
    path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-07_16-01-05-609625_0.4_min_path.ply")
    path_zyl_compl_2 = os.path.join(os.getcwd(), "output_new", "2024-06-11_10-45-54-282186_0.4_min_path.ply")
    path_zyl_compl_4 = os.path.join(os.getcwd(), "output_new", "2024-06-11_14-41-33-528331_0.4_min_path.ply")

    # read pointcloud and convert to array
    pcd = o3d.io.read_point_cloud(path_colon_min)

    
    line_bSpline = get_bSpline(pcd, sample_size)

    
    #p = line_bSpline.evaluate_single(get_closest_point_on_spline(pcd_colon, line_bSpline))

    # create point cloud from curve
    #print(p)
    """p_pcd = o3d.geometry.PointCloud()    
    p_pcd.points = o3d.utility.Vector3dVector(p)#p_np)
    p_pcd.paint_uniform_color([1,0,1])"""

    # convert Spline to pcd and export
    line_pcd = convert_bSpline_to_pcd(line_bSpline)
    line_pcd.paint_uniform_color([1,0,0])
    export_pcd_as_ply(line_pcd, "output_curve", f"curve_as_pointcloud-{sample_size}")

    colors_line = np.asarray(pcd_colon.colors)
    #colors_line[4000] = [1,0,0]
    pcd_colon.colors = o3d.utility.Vector3dVector(colors_line)
    # visualize
    o3d.visualization.draw_geometries([line_pcd, pcd_colon],#, p_pcd],
        mesh_show_wireframe = True,
        mesh_show_back_face = True,
        point_show_normal = True)
    
    vector_to_line, t_on_line, half_line = get_closest_point_on_spline(pcd_colon, line_bSpline)
    #print(f"vecot to line {t_on_line[0:1000]}")

    p_pcd = o3d.geometry.PointCloud()    
    p_pcd.points = o3d.utility.Vector3dVector(half_line)#p_np)
    p_pcd.paint_uniform_color([0,0,1])
    o3d.visualization.draw_geometries([line_pcd, pcd_colon, p_pcd],
        mesh_show_wireframe = False,
        mesh_show_back_face = False,
        point_show_normal = True)

    find_smallest_dis_to_point((pcd_colon), line_bSpline, vector_to_line, t_on_line)
    visualize(pcd_colon)


if __name__ == "__main__": main()  





"""
19.06.24
import geomdl.fitting
import numpy as np
import geomdl
import os
import open3d as o3d
import scipy
import matplotlib.pyplot as plt
import scipy.interpolate
from datetime import datetime
from time import sleep
import math
import plot_file

import copy 

def get_bSpline(pcd, sample_size):
    pcd_np = np.asarray(pcd.points)

    # interpolate line and upsample
    line_pc_array = np.ndarray.tolist(pcd_np)

    b_spline = geomdl.fitting.interpolate_curve(line_pc_array, 2)
    b_spline.sample_size = sample_size * b_spline.sample_size

    return b_spline


def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name = None):
    # get current date and time
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    # export as ply
    if dir_name != None:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, dir_name, f"{date_time}_{output_name_without_ply}.ply"), pcd)
    else:
        o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"), pcd)


def convert_bSpline_to_pcd(bSpline):
    # get points from bSpline
    points_bSpline = bSpline.evalpts
    points_bSpline_np = np.array(points_bSpline)

    # create point cloud from curve
    line_pcd = o3d.geometry.PointCloud()    
    line_pcd.points = o3d.utility.Vector3dVector(points_bSpline_np)

    return line_pcd


def visualize(pcd_colon):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    for i in range(100):
        pcd_colon.paint_uniform_color(change_color()[0])
        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(1)

    vis.register_animation_callback(move_colon)
    vis.run()
    vis.destroy_window()


def change_color():
    color = np.random.rand(1,3)
    print(color)
    
    return color
    #vis.get_view_Control()


def move_colon(vis):
    #color = np.random.rand([3,1])
    pass
    #vis.get_view_Control()

def get_closest_point_on_spline(pcd, bSpline, normals_to_inside):
    bSpline.evaluate()
    #print(f"splijne {len(bSpline)} and {bSpline.evaluate_single(0.501)}")
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    #t_old = 0.1
    #p = points[4000]
    #print(p[1])
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    half_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])
    #print(f" {np.linspace(0, 1, 20)} and {type(np.linspace(0, 1, 20))}")
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, 40)))
    #print(start_points)
    for i in range(np.shape(points)[0]):
        # approximate start value to be determined
        p = points[i]
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])

        max_dist_sorted = np.argsort(dist) 

        for s_p in range(0, np.shape(start_points)[0], 1):
            t_old = np.linspace(0, 1, 40)[max_dist_sorted[s_p]]
            for k in range(30):
                point_on_spline = bSpline.derivatives(t_old, 2)
                f_t = (point_on_spline[0][0] - p[0]) * point_on_spline[1][0] + (point_on_spline[0][1] - p[1]) * point_on_spline[1][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[1][2] 
                f__t = point_on_spline[1][0] * point_on_spline[1][0] + point_on_spline[1][1] * point_on_spline[1][1] + point_on_spline[1][2] * point_on_spline[1][2]#+ (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1]#+ point_on_spline[1][2] * point_on_spline[1][2] + (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[2][2]
                t = t_old - f_t / f__t    
    
                if t < 0.00:
                    t_old = 0.0
                elif t > 1.00:
                    t_old = 1.0
                else: 
                    t_old = t  

            vector = bSpline.evaluate_single(t_old) - p
            if normals_to_inside:
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) > - 0.7: # 45 ° in both directions
                    t_on_line[i] = t_old
                    vector_to_line[i] = vector
                    if i % 200 == 0:
                        print(f"{np.round(i / np.shape(points)[0], decimals=3) * 100} % done")
                    break
            else:
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) < 0.7: # 45 ° in both directions
                    t_on_line[i] = t_old
                    vector_to_line[i] = vector
                    if i % 20 == 0:
                        print(f"{np.round(i / np.shape(points)[0], decimals=3) * 100} % done")
                    break

        half_line[i] = p + 0.1 * vector_to_line[i]
        #print("here")

    l = bSpline.evaluate_single(0.042383170524102884)
    plot_file.plot_vectors(vector_to_line, points, start_points,l)
    #half_line = points + 0.5 * vector_to_line
    vector_to_line_distances = np.zeros(np.shape(vector_to_line)[0])
    
    vector_to_line_distances[:] = np.abs(np.linalg.norm(vector_to_line[:], axis=1))
    #print(f"vector_to line_dis {np.shape(vector_to_line_distances)} nicht shape {vector_to_line_distances[0:300]}")
    plot_vectors(t_on_line, vector_to_line_distances, 0)
    return vector_to_line, t_on_line, half_line, vector_to_line_distances


def plot_vectors(pcd_colon, vector_to_line,  start_points):
    fig = plt.figure()

    ax = fig.add_subplot()
    ax.scatter(pcd_colon, vector_to_line)
    #ax.quiver(pcd_colon[:,0], pcd_colon[:,1], pcd_colon[:,2], vector_to_line[:,0], vector_to_line[:,1], vector_to_line[:,2])
    #ax.plot(*start_points, color = 'r')
    # Set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    #ax.set_zlabel('z')

    plt.show()

def find_min_distances(vector_to_line_distances, t_on_line, bSpline):
    # if a t appears several times in t_on_line, just save the min distance for this t
    t_vec_combined = np.c_[t_on_line, vector_to_line_distances]
    u, c = np.unique(t_on_line, return_counts=True)
    multiple_arg = np.argwhere(c != 1)
    t_multiple_values = u[multiple_arg]

    for x in range(np.shape(t_multiple_values)[0]):
        vec_multi_arg = np.argwhere(t_vec_combined[:,0] == t_multiple_values[x][0])
        min = np.min(t_vec_combined[vec_multi_arg[:],1])
        t_vec_combined = np.delete(t_vec_combined, vec_multi_arg, axis=0)
        t_vec_combined = np.insert(t_vec_combined, 0, np.array([t_multiple_values[x][0], min]), axis=0)

    # sort by t
    ind = np.argsort(t_vec_combined[:,0] )
    t_vec_combined = t_vec_combined[ind]
    create_bins_find_local_mins(t_vec_combined)
    find_points_to_pull(t_vec_combined,vector_to_line_distances, t_on_line, bSpline)

    plot_vectors(t_vec_combined[:,0], t_vec_combined[:,1], 0)

def create_bins_find_local_mins(t_vec_combined):
    bin_size = 0.01
    min_bin_arg = np.zeros([1/bin_size])
    for b_s in range(1/bin_size):
        bin_arg = np.argwhere(((t_vec_combined[:,0] >= b_s) & (t_vec_combined[:,0] <= (b_s + bin_size))))
        min_bin_arg[b_s] = np.argmin(t_vec_combined[bin_arg[:],1])
        print(f"bin_arg {bin_arg}")

    print(f"min bin arg {min_bin_arg}")


def find_points_to_pull(t_vec_combined, vector_to_line_distances, t_on_line, bSpline):


    t_sort_indices = np.argsort(t_on_line)
    start_t = np.linspace(0, 1, 40)
    indexes = np.zeros(np.shape(start_t)[0])
    for start_p in range (np.shape(start_t)[0]):
        start = np.argmin(np.abs(t_vec_combined[:, 0] - start_t[start_p]))
        print(t_vec_combined[:,0])
        index_start = start
        print(f"start _ {start}")

        for i in range(3):
            if index_start == 0:
                index_start = 1

            m = (t_vec_combined[index_start, 1] - t_vec_combined[index_start - 1, 1]) / (t_vec_combined[index_start, 0] - t_vec_combined[index_start - 1, 0])
            print(f"asdf {t_sort_indices[index_start]}  and {t_on_line[t_sort_indices[index_start]]} and {m}")
            t_new = index_start - t_vec_combined[index_start, 1] / m
            index_start = np.argmin(np.abs(t_vec_combined[:, 0] - t_new))#t_vec_combined[index_start, 0] - m))
        indexes[start_p] = index_start
        
    print(f"ind {indexes}")
    fig = plt.figure()
    indexes = indexes.astype(int, copy = False)
    ax = fig.add_subplot()
    ax.scatter(t_vec_combined[:,0], t_vec_combined[:,1], color = 'y')
    ax.scatter(t_vec_combined[indexes[:],0], t_vec_combined[indexes[:],1], color= 'r')
    #ax.quiver(pcd_colon[:,0], pcd_colon[:,1], pcd_colon[:,2], vector_to_line[:,0], vector_to_line[:,1], vector_to_line[:,2])
    #ax.plot(*start_points, color = 'r')
    # Set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    """"""print(np.shape(point))
    fig = plt.figure()
    
    ax = fig.add_subplot()
    ax.scatter(point[:,0], point[:,1])"""
    #ax.scatter(t_on_line, vector_to_line_distances)
    #ax.scatter(t_on_line[t_sort_indices[indexes[:]]], vector_to_line_distances[t_sort_indices[indexes[:]]])
"""ax.set_xlabel('x')
    ax.set_ylabel('y')""""""

    plt.show()


def find_smallest_dis_to_point(pcd_colon, bspline, vector_to_line, t_on_line):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    print("there")
    points = copy.deepcopy(np.asarray(pcd_colon.points))
    z = 0
    # copy by reference
    half_line = np.asarray(pcd_colon.points)
    print("h")
    for i in range(500):
        dist_to_z = np.power(np.abs(1 - np.abs((z % 1.0) - t_on_line[:])), 4)
        dist_to_z = np.power(np.abs(1 - np.abs((z % 1.0) - t_on_line[:])), 4)
        #print(f"dist to z {dist_to_z}")
        
        #disc = np.argwhere(np.abs((z % 1.0) - t_on_line[:]) > 0.5)
        #print(f"shkpae {np.shape(disc)}")
        half_line[:, 0] = points[:, 0] + np.multiply(dist_to_z, vector_to_line[:, 0])
        half_line[:, 1] = points[:, 1] + np.multiply(dist_to_z, vector_to_line[:, 1])
        half_line[:, 2] = points[:, 2] + np.multiply(dist_to_z, vector_to_line[:, 2])
        #half_line[10] = points[10] + 0.25 * vector_to_line[10]
        #pcd_colon.points = o3d.utility.Vector3dVector(half_line)

        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(0.1)
        z+=0.01

        """"""half_line = points + 0.1* vector_to_line
        
        #half_line[10] = points[10] + 0.25 * vector_to_line[10]
        pcd_colon.points = o3d.utility.Vector3dVector(half_line)

        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(0.1)
        z+=0.001""""""
    #vis.register_animation_callback(move_colon)
    vis.run()
    vis.destroy_window()

def main():

    # adjustable parameter
    sample_size = 2
    normals_to_inside = False


    # prepare colon for visualization
    path_z_simple = os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_2= os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_4 = os.path.join(os.getcwd(), "data","zylinder_compl-4.ply")
    path_colon = os.path.join(os.getcwd(), "data", "Colon.ply")
    path_z_seg = os.path.join(os.getcwd(), "data", "colon_segments.ply")
    path_colon_sub = os.path.join(os.getcwd(), "data", "Colon_subtriangles_2.ply")
    pcd_colon = o3d.io.read_point_cloud(path_colon_sub)
    pcd_colon.paint_uniform_color([1,1,0])
    
    # data paths
    path_ = os.path.join(os.getcwd(), "output_new","2024-05-27_08-22-28-505194_line_2.ply")
    path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-28_16-13-50-420242_0.25_min_path.ply")
    path_colon_min = os.path.join(os.getcwd(), "output_new", "2024-06-03_10-23-22-088632_0.425_min_path.ply")#"2024-06-03_10-24-25-169883_0.4_min_path.ply")
    path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-07_16-01-05-609625_0.4_min_path.ply")
    path_zyl_compl_2 = os.path.join(os.getcwd(), "output_new", "2024-06-11_10-45-54-282186_0.4_min_path.ply")
    path_zyl_compl_4 = os.path.join(os.getcwd(), "output_new","output_new", "2024-06-11_14-41-33-528331_0.4_min_path.ply")
    path_zyl_seg = os.path.join(os.getcwd(),"output_main", "colon_segments__2024-06-17_11-31-52-383769", "2024-06-17_11-32-19-744015_0.3_min_path.ply")

    # read pointcloud and convert to array
    pcd = o3d.io.read_point_cloud(path_colon_min)

    
    line_bSpline = get_bSpline(pcd, sample_size)

    
    #p = line_bSpline.evaluate_single(get_closest_point_on_spline(pcd_colon, line_bSpline))

    # create point cloud from curve
    #print(p)
    p_pcd = o3d.geometry.PointCloud()    
    p_pcd.points = o3d.utility.Vector3dVector(p)#p_np)
    p_pcd.paint_uniform_color([1,0,1])

    # convert Spline to pcd and export
    line_pcd = convert_bSpline_to_pcd(line_bSpline)
    line_pcd.paint_uniform_color([1,0,0])
    #export_pcd_as_ply(line_pcd, "output_curve", f"curve_as_pointcloud-{sample_size}")

    colors_line = np.asarray(pcd_colon.colors)
    #colors_line[4000] = [1,0,0]
    pcd_colon.colors = o3d.utility.Vector3dVector(colors_line)
    # visualize
    o3d.visualization.draw_geometries([line_pcd, pcd_colon],#, p_pcd],
        mesh_show_wireframe = True,
        mesh_show_back_face = True,
        point_show_normal = True)
    
    vector_to_line, t_on_line, half_line, vector_to_line_distances = get_closest_point_on_spline(pcd_colon, line_bSpline, normals_to_inside)
    #print(f"vecot to line {t_on_line[0:1000]}")
    find_min_distances(vector_to_line_distances, t_on_line, line_bSpline)
    #find_points_to_pull(vector_to_line_distances, t_on_line, line_bSpline)

    p_pcd = o3d.geometry.PointCloud()    
    p_pcd.points = o3d.utility.Vector3dVector(half_line)#p_np)
    p_pcd.paint_uniform_color([0,0,1])
    o3d.visualization.draw_geometries([line_pcd, pcd_colon, p_pcd],
        mesh_show_wireframe = False,
        mesh_show_back_face = False,
        point_show_normal = True)

    find_smallest_dis_to_point((pcd_colon), line_bSpline, vector_to_line, t_on_line)
    #visualize(pcd_colon)


if __name__ == "__main__": main()  """

