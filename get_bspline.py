import geomdl.fitting
import numpy as np
import geomdl.exchange
import os
import open3d as o3d
import scipy
import matplotlib.pyplot as plt
import scipy.interpolate
from datetime import datetime
from time import sleep
import math
import plot_file
from scipy.signal import argrelmin

import copy 

def get_bSpline(pcd, sample_size):
    pcd_np = np.asarray(pcd.points)

    # interpolate line and upsample
    line_pc_array = np.ndarray.tolist(pcd_np)

    b_spline = geomdl.fitting.interpolate_curve(line_pc_array, 2)
    b_spline.sample_size = sample_size * b_spline.sample_size

    return b_spline


def export_pcd_as_ply(pcd, output_name_without_ply, folder):#output_folder, dir_name = None):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    # export as ply
    name = f"{date_time}_{output_name_without_ply}.ply"
    o3d.io.write_point_cloud(os.path.join(os.getcwd(), *folder, name), pcd)


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
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) > - 1: # 45 ° in both directions
                    t_on_line[i] = t_old
                    vector_to_line[i] = vector
                    if i % 200 == 0:
                        print(f"{np.round(i / np.shape(points)[0], decimals=3) * 100} % done")
                    break
            else:
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) < 1: # 45 ° in both directions
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

def find_min_distances(vector_to_line_distances, t_on_line, bSpline, pcd_colon, bin_size):
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
    local_mins = create_bins_find_local_mins(t_vec_combined, bin_size)
    plot_vectors(t_vec_combined[:,0], t_vec_combined[:,1], 0)

    #simulate_motion(bSpline, pcd_colon, t_vec_combined, min_distances, vector_to_line, t_on_line)

    return local_mins

def get_spline_length(b_spline):
    ctr_points = np.asarray(b_spline.ctrlpts)
    ctr_size = b_spline.ctrlpts_size
    appr_length_bspline = 0
    for i in range(ctr_size-1):
        appr_length_bspline += math.dist(ctr_points[i,:], ctr_points[i+1,:])

    average_length_vector = appr_length_bspline / ctr_size
    print(appr_length_bspline)
    
    return appr_length_bspline

def create_bins_find_local_mins(t_vec_combined, bin_size):
    #bin_size = 0.01
    min_bin_arg = np.zeros([int(1/bin_size)])
    for b_s in range(int(1/bin_size)):
        bin_arg = np.argwhere(((t_vec_combined[:,0] >= bin_size * b_s) & (t_vec_combined[:,0] <= (b_s * bin_size+ bin_size))))
        min_bin = np.argmin(t_vec_combined[bin_arg[:],1])
        min_t = (t_vec_combined[bin_arg[:], 0]) [min_bin]
        min_bin_arg[b_s] = np.argwhere(t_vec_combined[:, 0] == min_t)
        #print(f"bin_arg {bin_arg}")

    print(f"min bin arg {min_bin_arg}")
    fig = plt.figure()
    ax = fig.add_subplot()
    min_bin_arg_1 = min_bin_arg.astype(int)
    ax.scatter(t_vec_combined[:,0], t_vec_combined[:,1], color = 'y')
    ax.scatter(t_vec_combined[min_bin_arg_1[:],0], t_vec_combined[min_bin_arg_1[:],1], color= 'r')
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    plt.show()
    min_bins = t_vec_combined[min_bin_arg_1]
    y = t_vec_combined[min_bin_arg_1,1]
    arg = argrelmin(y,order=2)#np.shape(min_bin_arg)[0] // 8 )

    """line_pc_array = np.ndarray.tolist(t_vec_combined[min_bin_arg_1[:],:])

    b_spline = geomdl.fitting.interpolate_curve(line_pc_array, 2)
    b_spline.sample_size = b_spline.sample_size * 10
    x = np.asarray(b_spline.evalpts)
    print(f"x {np.shape(min_bins[arg[:],1])}")
    local_mins = np.zeros([int(1/bin_size)])
    for b_s in range(int(1/bin_size)):
        t_old = 5 * bin_size
        #b_s * 5#bin_size
        #print(f"t_old {t_old}")
        for k in range(50):
            point_on_spline = b_spline.derivatives(t_old, 2)
            f_t = (point_on_spline[0][0]) * point_on_spline[1][0] + (point_on_spline[0][1]) * point_on_spline[1][1]# + (point_on_spline[0][2]- p[2]) * point_on_spline[1][2] 
            f__t = point_on_spline[1][0] * point_on_spline[1][0] + point_on_spline[1][1] * point_on_spline[1][1]# + point_on_spline[1][2] * point_on_spline[1][2]#+ (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1]#+ point_on_spline[1][2] * point_on_spline[1][2] + (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[2][2]
            t_old = t_old - point_on_spline[1][1] / point_on_spline[2][1] * (point_on_spline[1][1] / point_on_spline[2][1])#ff_t / f__t  

            if t_old < 0.00:
                t_old = 0.0
            elif t_old > 1.00:
                t_old = 1.0
            else: 
                t_old = t_old  


    
        local_mins[b_s] = t_old
    
    local_mins = np.round(local_mins[:], decimals = 5)
    local_mins = np.unique(local_mins)
    print(f"local mins {local_mins}")
"""
    fig = plt.figure()
    ax = fig.add_subplot()
    min_bin_arg_1 = min_bin_arg.astype(int)
    ax.scatter(t_vec_combined[:,0], t_vec_combined[:,1], color = 'y')
    z = min_bins[arg[:]]
    ax.scatter(np.linspace(0, 1, int(1/bin_size)), np.full(int(1/bin_size), [0.25]), color = 'c')
    ax.scatter(t_vec_combined[min_bin_arg_1[:],0], t_vec_combined[min_bin_arg_1[:],1], color= 'r')
    ax.scatter(min_bins[arg[:],0], min_bins[arg[:],1], color='m')
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    

    plt.show()
    """fig = plt.figure()
    ax = fig.add_subplot()
    min_bin_arg_1 = min_bin_arg.astype(int)
    ax.scatter(t_vec_combined[:,0], t_vec_combined[:,1], color = 'y')
    ax.scatter(t_vec_combined[min_bin_arg_1[:],0], t_vec_combined[min_bin_arg_1[:],1], color= 'r')
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    plt.show()"""
    #print(local_mins)

    return z
            
    
    #b_spline.sample_size = sample_size * b_spline.sample_size

"""b_Spline: medial axis as bSpline
   pcd_colon: pointcloud of object
   t_vec_combined: array[t, distance]: shortest distance from pcd_colon points to medial axis, t = point on line
   min_distances: array[t, min_dist]: t at min distance
   vector_to_line: vector from pcd points to medial axis
   t_on_line: t to vector_to_line
"""
def simulate_motion(b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    points = copy.deepcopy(np.asarray(pcd_colon.points))
    # copy by reference
    moving_part = np.asarray(pcd_colon.points)
    for min_dist_ in range(np.shape(min_distances)[0]*6):
        min_dist = min_dist_% np.shape(min_distances)[0]


        t_smallest_distance_arg = np.argwhere(t_on_line == min_distances[min_dist, 0])
        contraction_point = np.asarray(b_Spline.evaluate_single(min_distances[min_dist, 0]))
        half_point = (points[t_smallest_distance_arg[0], :] + 0.5 * vector_to_line[t_smallest_distance_arg[0], :])[0]
        #print(f"cont {contraction_point}  abd half {half_point}")
        dist_half_contr_point = math.dist(contraction_point, half_point)

        if min_dist == 0:
            oral_half = 0
        else:
            oral_half = (min_distances[min_dist, 0] + min_distances[min_dist - 1, 0]) / 2
            
            if min_dist == 1:
                oral_half_old = 0
            else:
                oral_half_old = (min_distances[min_dist-1, 0] + min_distances[min_dist - 2, 0]) / 2

            aboral_half_old = (min_distances[min_dist-1, 0] + min_distances[min_dist, 0]) / 2

            t_contr_oral_arg_old = np.argwhere((t_on_line > oral_half_old) & (t_on_line <= min_distances[min_dist-1, 0]))
            t_contr_aboral_arg_old = np.argwhere((t_on_line > min_distances[min_dist-1, 0]) & (t_on_line < aboral_half_old))

            multi_factor_oral_old = 1 - (np.abs(min_distances[min_dist - 1, 0] - t_on_line[t_contr_oral_arg_old])) / np.abs(min_distances[min_dist-1, 0] - oral_half_old)
            multi_factor_aboral_old = 1 - (np.abs(min_distances[min_dist - 1, 0] - t_on_line[t_contr_aboral_arg_old])) / np.abs(min_distances[min_dist-1, 0] - aboral_half_old)
        
        if min_dist == np.shape(min_distances)[0] - 1:
            aboral_half = 1
        else:
            aboral_half = (min_distances[min_dist, 0] + min_distances[min_dist + 1, 0]) / 2

        t_contr_oral_arg = np.argwhere((t_on_line > oral_half) & (t_on_line <= min_distances[min_dist, 0]))
        t_contr_aboral_arg = np.argwhere((t_on_line > min_distances[min_dist, 0]) & (t_on_line < aboral_half))

        multi_factor_oral = 1 - (np.abs(min_distances[min_dist, 0] - t_on_line[t_contr_oral_arg])) / np.abs(min_distances[min_dist, 0] - oral_half)
        multi_factor_aboral = 1 - (np.abs(min_distances[min_dist, 0] - t_on_line[t_contr_aboral_arg])) / np.abs(min_distances[min_dist, 0] - aboral_half)

        while math.dist(contraction_point, moving_part[t_smallest_distance_arg[0]][0]) > dist_half_contr_point:
            moving_part[t_contr_oral_arg[:,0]] += 0.01 * multi_factor_oral * vector_to_line[t_contr_oral_arg[:,0]]
            moving_part[t_contr_aboral_arg[:,0]] += 0.01 * multi_factor_aboral * vector_to_line[t_contr_aboral_arg[:,0]]
            
            if min_dist != 0:
                moving_part[t_contr_oral_arg_old[:,0]] -= 0.01 * multi_factor_oral_old* vector_to_line[t_contr_oral_arg_old[:,0]]
                moving_part[t_contr_aboral_arg_old[:,0]] -= 0.01 * multi_factor_aboral_old* vector_to_line[t_contr_aboral_arg_old[:,0]]

            vis.poll_events()
            vis.update_geometry(pcd_colon)
            vis.update_renderer()
            sleep(0.03)

        if min_dist == np.shape(min_distances)[0] - 1:
            orig_point = (points[t_smallest_distance_arg[0], :])[0]
            dist_orig_contr_point = math.dist(contraction_point, orig_point)

            while math.dist(contraction_point, moving_part[t_smallest_distance_arg[0]][0]) < dist_orig_contr_point:

                moving_part[t_contr_oral_arg[:,0]] -= 0.01 * multi_factor_oral * vector_to_line[t_contr_oral_arg[:,0]]
                moving_part[t_contr_aboral_arg[:,0]] -= 0.01 * multi_factor_aboral * vector_to_line[t_contr_aboral_arg[:,0]]
                
                vis.poll_events()
                vis.update_geometry(pcd_colon)
                vis.update_renderer()
                sleep(0.03)

    vis.run()
    vis.destroy_window()

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
            index_start = np.argmin(np.abs(t_vec_combined[:, 0] - t_new))
        indexes[start_p] = index_start
        
    print(f"ind {indexes}")
    fig = plt.figure()
    indexes = indexes.astype(int, copy = False)
    ax = fig.add_subplot()
    ax.scatter(t_vec_combined[:,0], t_vec_combined[:,1], color = 'y')
    ax.scatter(t_vec_combined[indexes[:],0], t_vec_combined[indexes[:],1], color= 'r')
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    plt.show()


def find_smallest_dis_to_point(pcd_colon, bspline, vector_to_line, t_on_line, eval_points):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    points = copy.deepcopy(np.asarray(pcd_colon.points))
    z = 0
    # copy by reference
    half_line = np.asarray(pcd_colon.points)
    for i in range(np.shape(eval_points)[0]):
        dist_to_z = np.power(np.abs(1 - np.abs((eval_points[i] % 1.0) - t_on_line[:])), 4)
        dist_to_z = np.power(np.abs(1 - np.abs((eval_points[i] % 1.0) - t_on_line[:])), 4)

        half_line[:, 0] = points[:, 0] + np.multiply(dist_to_z, vector_to_line[:, 0])
        half_line[:, 1] = points[:, 1] + np.multiply(dist_to_z, vector_to_line[:, 1])
        half_line[:, 2] = points[:, 2] + np.multiply(dist_to_z, vector_to_line[:, 2])

        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(1)
        z+=0.01

    vis.run()
    vis.destroy_window()

def export_spline_as_json(b_spline, output_name_without_json, folder):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    # export as ply
    name = f"{date_time}_{output_name_without_json}.json"
    geomdl.exchange.export_json(b_spline, os.path.join(os.getcwd(), *folder, name))

    return name

def main():

    # adjustable parameter
    sample_size = 2
    normals_to_inside = True


    # prepare colon for visualization
    path_z_simple = os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_2= os.path.join(os.getcwd(), "data","zylinder_compl-2.ply")
    path_z_compl_4 = os.path.join(os.getcwd(), "data","zylinder_compl-4.ply")
    path_colon = os.path.join(os.getcwd(), "data", "Colon.ply")
    path_z_seg = os.path.join(os.getcwd(), "data", "colon_segments.ply")
    path_colon_sub = os.path.join(os.getcwd(), "data", "Colon_subtriangles_2.ply")
    path_z_more_seg = os.path.join(os.getcwd(), "data", "colon_segments_more_complicated.ply") 
    pcd_colon = o3d.io.read_point_cloud(path_z_more_seg)
    pcd_colon.paint_uniform_color([1,1,0])

    data = "colon_seg_more_compl"

    pcd_colon_1 = o3d.io.read_point_cloud(path_z_more_seg)
    pcd_colon_1.paint_uniform_color([1,1,0])
    
    # data paths
    path_ = os.path.join(os.getcwd(), "output_new","2024-05-27_08-22-28-505194_line_2.ply")
    path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-28_16-13-50-420242_0.25_min_path.ply")
    path_colon_min = os.path.join(os.getcwd(), "output_new", "2024-06-03_10-23-22-088632_0.425_min_path.ply")#"2024-06-03_10-24-25-169883_0.4_min_path.ply")
    path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-07_16-01-05-609625_0.4_min_path.ply")
    path_zyl_compl_2 = os.path.join(os.getcwd(), "output_new", "2024-06-11_10-45-54-282186_0.4_min_path.ply")
    path_zyl_compl_4 = os.path.join(os.getcwd(), "output_new","output_new", "2024-06-11_14-41-33-528331_0.4_min_path.ply")
    path_zyl_seg = os.path.join(os.getcwd(),"output_main", "colon_segments__2024-06-17_11-31-52", "2024-06-17_11-32-19-744015_0.3_min_path.ply")
    path_zyl_seg_compl = os.path.join(os.getcwd(),"output_main", "colon_segments__2024-06-17_11-31-52", "2024-06-17_11-32-19-744015_0.3_min_path.ply")
    path_zyl_seg_more_compl = os.path.join(os.getcwd(),"output_main", "colon_segments_more_complicated__2024-06-25_10-19-21", "2024-06-25_10-19-58-397080_0.2_min_path.ply")
    # read pointcloud and convert to array
    pcd = o3d.io.read_point_cloud(path_zyl_seg_more_compl)
    print((path_1[1]))

    o3d.visualization.draw_geometries([pcd_colon],
        mesh_show_wireframe = True,
        mesh_show_back_face = True,
        point_show_normal = True)
    
    normals_to_inside = input("Are normals pointing to inside? yes: insert True, no: insert False:  ")



    line_bSpline = get_bSpline(pcd, sample_size)
    export_spline_as_json(line_bSpline, f"{sample_size}_bSpline_medial_axis_{data}", ["output_curve"])

    length_spline = get_spline_length(line_bSpline)
    #p = line_bSpline.evaluate_single(get_closest_point_on_spline(pcd_colon, line_bSpline))

    # create point cloud from curve
    #print(p)
    """p_pcd = o3d.geometry.PointCloud()    
    p_pcd.points = o3d.utility.Vector3dVector(p)#p_np)
    p_pcd.paint_uniform_color([1,0,1])"""

    # convert Spline to pcd and export
    line_pcd = convert_bSpline_to_pcd(line_bSpline)
    line_pcd.paint_uniform_color([1,0,0])
    outputfolder = ["output_curve"]
    export_pcd_as_ply(line_pcd, f"curve_as_pointcloud-{sample_size}", outputfolder)

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
    bin_size = 0.2 / length_spline
    local_mins = find_min_distances(vector_to_line_distances, t_on_line, line_bSpline, pcd_colon, bin_size)
    #find_points_to_pull(vector_to_line_distances, t_on_line, line_bSpline)

    simulate_motion(line_bSpline, pcd_colon, local_mins, vector_to_line, t_on_line)

    p_pcd = o3d.geometry.PointCloud()  
    print(type(half_line))  
    p_pcd.points = o3d.utility.Vector3dVector(half_line)#p_np)
    p_pcd.paint_uniform_color([0,0,1])

    p_p= o3d.geometry.PointCloud()    
    p_p.points = o3d.utility.Vector3dVector(line_bSpline.evaluate_list(local_mins[:,0]))#p_np)
    p_p.paint_uniform_color([0,0,1])

    o3d.visualization.draw_geometries([line_pcd, pcd_colon_1,p_p],
        mesh_show_wireframe = False,
        mesh_show_back_face = False,
        point_show_normal = True)

    find_smallest_dis_to_point((pcd_colon), line_bSpline, vector_to_line, t_on_line, local_mins[:,0])
    #visualize(pcd_colon)


if __name__ == "__main__": main()  
